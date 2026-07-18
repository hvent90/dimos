// Global error handler
window.onerror = (msg, url, line, col, error) => {
    console.error(`[ERROR] ${msg} at ${url}:${line}:${col}`, error);
    document.getElementById('status').textContent = `Error: ${msg}`;
};

import { geometry_msgs, std_msgs, sensor_msgs } from "https://esm.sh/jsr/@dimos/msgs@0.1.4";

// LCM fingerprint (first 8 bytes) of a UInt32, used to recognise inbound replay
// progress messages vs JPEG frames. Computed once by encoding a sample.
let PROGRESS_FINGERPRINT = null;
try {
    PROGRESS_FINGERPRINT = new Uint8Array(new std_msgs.UInt32({ data: 0 }).encode()).slice(0, 8);
} catch (e) {
    console.error('could not compute UInt32 fingerprint', e);
}

function matchesFingerprint(buf, fp) {
    if (buf.byteLength < fp.length) return false;
    const head = new Uint8Array(buf, 0, fp.length);
    for (let i = 0; i < fp.length; i++) if (head[i] !== fp[i]) return false;
    return true;
}

// Fraction [0,1] of the drawn line already consumed by an in-progress replay.
let replayProgress = 0;
function onReplayProgress(fraction) {
    replayProgress = Math.max(0, Math.min(1, fraction));
    if (replayProgress >= 1) {
        // Replay done — clear the line so it fully vanishes.
        clearLine();
        replayProgress = 0;
    }
}

// WebSocket and VR state
let ws = null;
let xrSession = null;
let xrRefSpace = null;
let gl = null;
let lastSendTime = 0;
const sendInterval = 1000 / 80; // ~80Hz target

// Video panel state
const videoEl = document.getElementById('videoFeed');
let videoTex = null;
let videoProgram = null;
let videoVbo = null;
let videoAttribs = null;
let videoUniforms = null;
let videoReady = false;  // true after first frame loads
let videoDirty = false;  // true when a new JPEG has finished decoding
let videoAspect = 1.0;   // cached at load time — see videoEl.onload
let prevBlobUrl = null;  // revoked when the next-next blob arrives
const videoModelMatrix = new Float32Array(16);

const PANEL_POS_X = 0.0;
const PANEL_POS_Y = 1.4;   // ~eye height
const PANEL_POS_Z = -1.5;  // 1.5m in front of starting position
const PANEL_HEIGHT = 0.9;

// Line-drawing state — the trajectory the operator sketches while holding grip.
// Points are captured in the same local-floor reference space used to render,
// so u_model is identity and the line floats in passthrough where drawn.
let lineProgram = null;
let lineVbo = null;
let ribbonVbo = null;
let axesVbo = null;
let lineAttribs = null;
let lineUniforms = null;
const LINE_MAX_POINTS = 4096;
const lineVerts = new Float32Array(LINE_MAX_POINTS * 3);  // xyz per point
// Ribbon: 2 triangles per drawn point (as a TRIANGLE_STRIP), 3 floats each.
const ribbonVerts = new Float32Array(LINE_MAX_POINTS * 2 * 3);
let lineCount = 0;         // points currently in the buffer
let lineDirty = false;     // true when new points need re-upload to the VBO
let drawing = false;       // true while the grip button is held
// Half-width of the drawn line, in metres. The ribbon is built camera-facing so
// this reads as a roughly constant thickness regardless of viewing angle.
// Tuned for how it looks in the passthrough video — nudge to taste.
const LINE_HALF_WIDTH = 0.0035;
const identityMatrix = new Float32Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
]);
// Only append a point once the controller has moved this far (metres) from the
// last one — keeps the polyline from piling up thousands of near-duplicate verts.
const LINE_MIN_STEP = 0.005;

// UI elements
const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connectBtn');
const disconnectBtn = document.getElementById('disconnectBtn');
const canvas = document.getElementById('canvas');

function setStatus(msg) {
    statusEl.textContent = msg;
}

// WebSocket setup (LCM bridge)
function setupWebSocket() {
    return new Promise((resolve, reject) => {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws`;

        setStatus('Connecting to server...');
        ws = new WebSocket(wsUrl);
        // arraybuffer so we can peek the first 8 bytes (LCM fingerprint) to tell
        // a small control message (replay progress) from a JPEG video frame.
        ws.binaryType = 'arraybuffer';

        ws.onopen = () => {
            setStatus('Server connected');
            resolve();
        };
        ws.onerror = (error) => {
            setStatus('WebSocket error');
            console.error('WebSocket error:', error);
            reject(error);
        };
        ws.onclose = () => {
            setStatus('WebSocket closed');
        };
        ws.onmessage = (e) => {
            const buf = e.data;
            if (!(buf instanceof ArrayBuffer)) return;
            // Dispatch by LCM fingerprint: a UInt32 is replay progress; anything
            // else is a JPEG frame (JPEG starts with 0xFFD8, never our fingerprint).
            if (PROGRESS_FINGERPRINT && matchesFingerprint(buf, PROGRESS_FINGERPRINT)) {
                try {
                    const permille = std_msgs.UInt32.decode(new Uint8Array(buf)).data;
                    onReplayProgress(permille / 1000);
                } catch (err) {
                    console.error('progress decode failed', err);
                }
                return;
            }
            // JPEG path: wrap the bytes back into a Blob for the <img> loader.
            const newUrl = URL.createObjectURL(new Blob([buf], { type: 'image/jpeg' }));
            if (prevBlobUrl) URL.revokeObjectURL(prevBlobUrl);
            prevBlobUrl = videoEl.src.startsWith('blob:') ? videoEl.src : null;
            videoEl.src = newUrl;
        };
    });
}

// Initialize WebGL
function initGL() {
    gl = canvas.getContext('webgl', {
        xrCompatible: true,
        alpha: true
    });
    if (!gl) {
        throw new Error('WebGL not supported');
    }
    gl.clearColor(0, 0, 0, 0); // Transparent background for passthrough
    initVideoPanel();
    initLine();
}

// Compile + link a plain colored-line pipeline for the drawn trajectory.
// Vertices are world-space (local-floor) points, so u_model is identity.
function initLine() {
    const vsSrc = `
        attribute vec3 a_pos;
        uniform mat4 u_proj;
        uniform mat4 u_view;
        uniform mat4 u_model;
        uniform float u_pointSize;
        void main() {
            gl_Position = u_proj * u_view * u_model * vec4(a_pos, 1.0);
            gl_PointSize = u_pointSize;
        }`;
    const fsSrc = `
        precision mediump float;
        uniform vec4 u_color;
        void main() {
            gl_FragColor = u_color;
        }`;

    const compile = (type, src) => {
        const sh = gl.createShader(type);
        gl.shaderSource(sh, src);
        gl.compileShader(sh);
        if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
            throw new Error('Line shader compile failed: ' + gl.getShaderInfoLog(sh));
        }
        return sh;
    };
    const vs = compile(gl.VERTEX_SHADER, vsSrc);
    const fs = compile(gl.FRAGMENT_SHADER, fsSrc);
    lineProgram = gl.createProgram();
    gl.attachShader(lineProgram, vs);
    gl.attachShader(lineProgram, fs);
    gl.linkProgram(lineProgram);
    if (!gl.getProgramParameter(lineProgram, gl.LINK_STATUS)) {
        throw new Error('Line program link failed: ' + gl.getProgramInfoLog(lineProgram));
    }

    lineVbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, lineVbo);
    // Allocate the full buffer up front; we bufferSubData the used prefix each frame.
    gl.bufferData(gl.ARRAY_BUFFER, lineVerts.byteLength, gl.DYNAMIC_DRAW);

    // Ribbon buffer: rebuilt on the CPU each frame (needs the camera position),
    // so it's DYNAMIC_DRAW and allocated to the worst case.
    ribbonVbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, ribbonVbo);
    gl.bufferData(gl.ARRAY_BUFFER, ribbonVerts.byteLength, gl.DYNAMIC_DRAW);

    lineAttribs = { pos: gl.getAttribLocation(lineProgram, 'a_pos') };
    lineUniforms = {
        proj:  gl.getUniformLocation(lineProgram, 'u_proj'),
        view:  gl.getUniformLocation(lineProgram, 'u_view'),
        model: gl.getUniformLocation(lineProgram, 'u_model'),
        color: gl.getUniformLocation(lineProgram, 'u_color'),
        pointSize: gl.getUniformLocation(lineProgram, 'u_pointSize'),
    };

    // World-axis gizmo at the origin: X/Y/Z as 0.3 m segments (red/green/blue),
    // so the operator always has an orientation reference regardless of how the
    // headset is turned. Static geometry, drawn with the same shader.
    axesVbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, axesVbo);
    const L = 0.3;
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
        0, 0, 0,  L, 0, 0,   // X
        0, 0, 0,  0, L, 0,   // Y
        0, 0, 0,  0, 0, L,   // Z
    ]), gl.STATIC_DRAW);
}

function renderAxes(view, viewport) {
    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(lineProgram);
    gl.bindBuffer(gl.ARRAY_BUFFER, axesVbo);
    gl.enableVertexAttribArray(lineAttribs.pos);
    gl.vertexAttribPointer(lineAttribs.pos, 3, gl.FLOAT, false, 0, 0);
    gl.uniformMatrix4fv(lineUniforms.proj,  false, view.projectionMatrix);
    gl.uniformMatrix4fv(lineUniforms.view,  false, view.transform.inverse.matrix);
    gl.uniformMatrix4fv(lineUniforms.model, false, identityMatrix);
    gl.uniform1f(lineUniforms.pointSize, 1.0);

    const depthWasOn = gl.isEnabled(gl.DEPTH_TEST);
    gl.disable(gl.DEPTH_TEST);
    gl.uniform4f(lineUniforms.color, 1.0, 0.2, 0.2, 1.0); // X red
    gl.drawArrays(gl.LINES, 0, 2);
    gl.uniform4f(lineUniforms.color, 0.2, 1.0, 0.2, 1.0); // Y green
    gl.drawArrays(gl.LINES, 2, 2);
    gl.uniform4f(lineUniforms.color, 0.3, 0.5, 1.0, 1.0); // Z blue
    gl.drawArrays(gl.LINES, 4, 2);
    if (depthWasOn) gl.enable(gl.DEPTH_TEST);
}

// Reset the drawn line (called on a fresh grip-press).
function clearLine() {
    lineCount = 0;
    lineDirty = true;
}

// Append a world-space point if it's far enough from the last one.
function appendLinePoint(x, y, z) {
    if (lineCount >= LINE_MAX_POINTS) return;
    if (lineCount > 0) {
        const i = (lineCount - 1) * 3;
        const dx = x - lineVerts[i];
        const dy = y - lineVerts[i + 1];
        const dz = z - lineVerts[i + 2];
        if (dx * dx + dy * dy + dz * dz < LINE_MIN_STEP * LINE_MIN_STEP) return;
    }
    const o = lineCount * 3;
    lineVerts[o] = x;
    lineVerts[o + 1] = y;
    lineVerts[o + 2] = z;
    lineCount++;
    lineDirty = true;
}

// Build a camera-facing ribbon (TRIANGLE_STRIP) from the polyline points in
// [startIdx, lineCount). Each point becomes two vertices offset ± a "side"
// vector perpendicular to both the segment and the view direction, so the line
// keeps a roughly constant on-screen thickness. Returns the vertex count.
function buildRibbon(startIdx, camX, camY, camZ) {
    let v = 0;
    for (let i = startIdx; i < lineCount; i++) {
        const p = i * 3;
        const px = lineVerts[p], py = lineVerts[p + 1], pz = lineVerts[p + 2];

        // Tangent: direction to the next point (or previous for the last one).
        let tx, ty, tz;
        if (i < lineCount - 1) {
            const n = (i + 1) * 3;
            tx = lineVerts[n] - px; ty = lineVerts[n + 1] - py; tz = lineVerts[n + 2] - pz;
        } else {
            const m = (i - 1) * 3;
            tx = px - lineVerts[m]; ty = py - lineVerts[m + 1]; tz = pz - lineVerts[m + 2];
        }

        // View vector from the point toward the camera.
        const vx = camX - px, vy = camY - py, vz = camZ - pz;

        // side = normalize(tangent × view) — perpendicular to both, i.e. the
        // screen-horizontal offset that makes the ribbon face the camera.
        let sx = ty * vz - tz * vy;
        let sy = tz * vx - tx * vz;
        let sz = tx * vy - ty * vx;
        const len = Math.hypot(sx, sy, sz);
        if (len > 1e-6) { sx /= len; sy /= len; sz /= len; }
        sx *= LINE_HALF_WIDTH; sy *= LINE_HALF_WIDTH; sz *= LINE_HALF_WIDTH;

        ribbonVerts[v++] = px - sx; ribbonVerts[v++] = py - sy; ribbonVerts[v++] = pz - sz;
        ribbonVerts[v++] = px + sx; ribbonVerts[v++] = py + sy; ribbonVerts[v++] = pz + sz;
    }
    return v / 3;
}

function renderLine(view, viewport) {
    if (lineCount < 2) return;

    // During replay, skip the consumed prefix so the line vanishes as the arm
    // retraces it — only the not-yet-played tail is drawn.
    const startIdx = Math.min(lineCount - 2, Math.floor(replayProgress * lineCount));
    if (lineCount - startIdx < 2) return;

    // Camera world position for this eye (last column of the view transform).
    const cam = view.transform.position;
    const nVerts = buildRibbon(startIdx, cam.x, cam.y, cam.z);
    if (nVerts < 3) return;

    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(lineProgram);

    gl.bindBuffer(gl.ARRAY_BUFFER, ribbonVbo);
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, ribbonVerts.subarray(0, nVerts * 3));
    gl.enableVertexAttribArray(lineAttribs.pos);
    gl.vertexAttribPointer(lineAttribs.pos, 3, gl.FLOAT, false, 0, 0);

    gl.uniformMatrix4fv(lineUniforms.proj,  false, view.projectionMatrix);
    gl.uniformMatrix4fv(lineUniforms.view,  false, view.transform.inverse.matrix);
    gl.uniformMatrix4fv(lineUniforms.model, false, identityMatrix);
    gl.uniform4f(lineUniforms.color, 0.1, 1.0, 0.3, 1.0);  // bright green
    gl.uniform1f(lineUniforms.pointSize, 1.0);

    // Overlay on top of passthrough + video panel.
    const depthWasOn = gl.isEnabled(gl.DEPTH_TEST);
    gl.disable(gl.DEPTH_TEST);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, nVerts);
    if (depthWasOn) gl.enable(gl.DEPTH_TEST);
}

// Compile + link a textured-quad pipeline that renders the camera
// feed as a world-locked panel inside the WebXR scene.
function initVideoPanel() {
    const vsSrc = `
        attribute vec2 a_pos;
        attribute vec2 a_uv;
        uniform mat4 u_proj;
        uniform mat4 u_view;
        uniform mat4 u_model;
        varying vec2 v_uv;
        void main() {
            gl_Position = u_proj * u_view * u_model
                        * vec4(a_pos.x, a_pos.y, 0.0, 1.0);
            v_uv = a_uv;
        }`;
    const fsSrc = `
        precision mediump float;
        varying vec2 v_uv;
        uniform sampler2D u_tex;
        void main() {
            gl_FragColor = texture2D(u_tex, v_uv);
        }`;

    const compile = (type, src) => {
        const sh = gl.createShader(type);
        gl.shaderSource(sh, src);
        gl.compileShader(sh);
        if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
            throw new Error('Shader compile failed: ' + gl.getShaderInfoLog(sh));
        }
        return sh;
    };
    const vs = compile(gl.VERTEX_SHADER, vsSrc);
    const fs = compile(gl.FRAGMENT_SHADER, fsSrc);
    videoProgram = gl.createProgram();
    gl.attachShader(videoProgram, vs);
    gl.attachShader(videoProgram, fs);
    gl.linkProgram(videoProgram);
    if (!gl.getProgramParameter(videoProgram, gl.LINK_STATUS)) {
        throw new Error('Program link failed: ' + gl.getProgramInfoLog(videoProgram));
    }

    // Quad as TRIANGLE_STRIP: x, y, u, v. v=0 at top of quad +
    // UNPACK_FLIP_Y_WEBGL=false → image displays upright.
    const verts = new Float32Array([
        -1, -1, 0, 1,
         1, -1, 1, 1,
        -1,  1, 0, 0,
         1,  1, 1, 0,
    ]);
    videoVbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, videoVbo);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);

    videoAttribs = {
        pos: gl.getAttribLocation(videoProgram, 'a_pos'),
        uv:  gl.getAttribLocation(videoProgram, 'a_uv'),
    };
    videoUniforms = {
        proj:  gl.getUniformLocation(videoProgram, 'u_proj'),
        view:  gl.getUniformLocation(videoProgram, 'u_view'),
        model: gl.getUniformLocation(videoProgram, 'u_model'),
        tex:   gl.getUniformLocation(videoProgram, 'u_tex'),
    };

    videoTex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, videoTex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);

    // Cache aspect here — naturalWidth can transiently drop to 0
    // between loads, which would collapse the panel to 1:1.
    videoEl.onload = () => {
        videoReady = true;
        videoDirty = true;
        if (videoEl.naturalHeight) {
            videoAspect = videoEl.naturalWidth / videoEl.naturalHeight;
        }
    };
}

function uploadVideoTexture() {
    gl.bindTexture(gl.TEXTURE_2D, videoTex);
    gl.texImage2D(
        gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, videoEl
    );
}

function updateModelMatrix() {
    const halfH = PANEL_HEIGHT * 0.5;
    const halfW = halfH * videoAspect;
    videoModelMatrix.fill(0);
    videoModelMatrix[0] = halfW;
    videoModelMatrix[5] = halfH;
    videoModelMatrix[10] = 1;
    videoModelMatrix[12] = PANEL_POS_X;
    videoModelMatrix[13] = PANEL_POS_Y;
    videoModelMatrix[14] = PANEL_POS_Z;
    videoModelMatrix[15] = 1;
}

function renderVideoPanel(view, viewport) {
    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(videoProgram);

    gl.bindBuffer(gl.ARRAY_BUFFER, videoVbo);
    gl.enableVertexAttribArray(videoAttribs.pos);
    gl.vertexAttribPointer(videoAttribs.pos, 2, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(videoAttribs.uv);
    gl.vertexAttribPointer(videoAttribs.uv,  2, gl.FLOAT, false, 16, 8);

    gl.uniformMatrix4fv(videoUniforms.proj,  false, view.projectionMatrix);
    gl.uniformMatrix4fv(videoUniforms.view,  false, view.transform.inverse.matrix);
    gl.uniformMatrix4fv(videoUniforms.model, false, videoModelMatrix);

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, videoTex);
    gl.uniform1i(videoUniforms.tex, 0);

    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
}


// Send raw controller tracking data (no processing - done in Python)
function processTracking(frame) {
    // Rate limit tracking data
    const now = performance.now();
    if (now - lastSendTime < sendInterval) {
        return;
    }
    lastSendTime = now;

    // Process input sources (controllers)
    for (const inputSource of frame.session.inputSources) {
        const trackingSpace = inputSource.gripSpace || inputSource.targetRaySpace;
        if (!trackingSpace) continue;

        const handedness = inputSource.handedness;
        if (handedness !== 'left' && handedness !== 'right') continue;

        const pose = frame.getPose(trackingSpace, xrRefSpace);
        if (!pose) continue;

        // Send raw pose directly from WebXR - no processing
        const pos = pose.transform.position;
        const rot = pose.transform.orientation;

        // Line drawing: while the RIGHT grip is held, record the controller's
        // world-space (local-floor) position into the drawn polyline. This is
        // purely visual — the backend records its own delta trajectory from the
        // same grip window. Grip is gamepad.buttons[1].
        if (handedness === 'right') {
            const gp = inputSource.gamepad;
            const drawHeld = !!(gp && gp.buttons[1] && gp.buttons[1].pressed);
            if (drawHeld && !drawing) {
                clearLine();          // fresh line on each new grip-press
                drawing = true;
                console.log('[line] collection STARTED (grip)');
            } else if (!drawHeld && drawing) {
                drawing = false;
                console.log('[line] collection ENDED — ' + lineCount + ' points');
            }
            if (drawing) {
                appendLinePoint(pos.x, pos.y, pos.z);
            }
        }

        const nowMs = Date.now();
        const poseStamped = new geometry_msgs.PoseStamped({
            header: new std_msgs.Header({
                stamp: new std_msgs.Time({ sec: Math.floor(nowMs / 1000), nsec: (nowMs % 1000) * 1_000_000 }),
                frame_id: handedness
            }),
            pose: new geometry_msgs.Pose({
                position: new geometry_msgs.Point({ x: pos.x, y: pos.y, z: pos.z }),
                orientation: new geometry_msgs.Quaternion({ x: rot.x, y: rot.y, z: rot.z, w: rot.w })
            })
        });

        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(poseStamped.encode());
        }

        // Send Joy message with all buttons and axes
        const gamepad = inputSource.gamepad;
        if (gamepad) {
            const isXrStandard = gamepad.mapping === 'xr-standard';
            const stickX = (isXrStandard ? gamepad.axes[2] : gamepad.axes[0]) ?? 0.0;
            const stickY = (isXrStandard ? gamepad.axes[3] : gamepad.axes[1]) ?? 0.0;
            const axes = [
                stickX,
                stickY,
                gamepad.buttons[0]?.value ?? 0.0,
                gamepad.buttons[1]?.value ?? 0.0,
            ];

            // Buttons layout (int32, 0 or 1):
            // [0] = trigger (digital)
            // [1] = grip (digital)
            // [2] = touchpad press
            // [3] = thumbstick press
            // [4] = X/A button
            // [5] = Y/B button
            // [6] = menu (if exposed)
            const buttons = [];
            for (let i = 0; i < gamepad.buttons.length; i++) {
                buttons.push(gamepad.buttons[i]?.pressed ? 1 : 0);
            }

            const joyMsg = new sensor_msgs.Joy({
                header: new std_msgs.Header({
                    stamp: new std_msgs.Time({ sec: Math.floor(nowMs / 1000), nsec: (nowMs % 1000) * 1_000_000 }),
                    frame_id: handedness
                }),
                axes_length: axes.length,
                buttons_length: buttons.length,
                axes: axes,
                buttons: buttons
            });

            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(joyMsg.encode());
            }
        }
    }
}

// VR render loop
function onXRFrame(_time, frame) {
    if (!xrSession) return;
    xrSession.requestAnimationFrame(onXRFrame);
    // Process and send tracking data
    processTracking(frame);

    const glLayer = xrSession.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, glLayer.framebuffer);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // Upload a fresh video frame if one arrived (line renders regardless).
    if (videoReady && videoDirty && videoEl.naturalWidth) {
        uploadVideoTexture();
        videoDirty = false;
    }
    updateModelMatrix();
    const pose = frame.getViewerPose(xrRefSpace);
    if (pose) {
        for (const view of pose.views) {
            const viewport = glLayer.getViewport(view);
            if (videoReady) renderVideoPanel(view, viewport);
            renderAxes(view, viewport);
            renderLine(view, viewport);
        }
    }
}

// Start VR session with passthrough
async function startVR() {
    try {
        setStatus('Initializing WebGL...');
        initGL();
        setStatus('Requesting VR session...');

        // Try immersive-ar first (true passthrough), fall back to immersive-vr
        let session = null;
        try {
            session = await navigator.xr.requestSession('immersive-ar', {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['hand-tracking']
            });
            console.log('Started immersive-ar session (passthrough)');
        } catch (arError) {
            console.log('immersive-ar not available, trying immersive-vr');
            session = await navigator.xr.requestSession('immersive-vr', {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['hand-tracking']
            });
            console.log('Started immersive-vr session');
        }

        xrSession = session;

        // Setup WebGL layer
        const glLayer = new XRWebGLLayer(session, gl);
        await session.updateRenderState({
            baseLayer: glLayer
        });

        // Get reference space
        xrRefSpace = await session.requestReferenceSpace('local-floor');

        setStatus('VR active');

        // Session event handlers
        session.addEventListener('end', () => {
            setStatus('VR session ended');
            xrSession = null;
            window.disconnect();
        });

        // Start render loop
        session.requestAnimationFrame(onXRFrame);

    } catch (error) {
        setStatus('VR failed: ' + error.message);
        console.error('VR session error:', error);
        throw error;
    }
}

// Connect button handler
window.connect = async function() {
    try {
        connectBtn.disabled = true;

        // Check WebXR support
        if (!navigator.xr) {
            throw new Error('WebXR not supported. Use Quest 3 browser.');
        }

        // Setup WebSocket
        await setupWebSocket();

        // Start VR
        await startVR();

        // Update UI
        connectBtn.classList.add('hidden');
        disconnectBtn.classList.remove('hidden');

    } catch (error) {
        setStatus('Connection failed');
        console.error('Connection error:', error);
        connectBtn.disabled = false;
    }
};

// Disconnect button handler
window.disconnect = async function() {
    setStatus('Disconnecting...');

    if (xrSession) {
        await xrSession.end().catch(console.error);
        xrSession = null;
    }

    if (ws) {
        ws.close();
        ws = null;
    }

    // Update UI
    connectBtn.classList.remove('hidden');
    connectBtn.disabled = false;
    disconnectBtn.classList.add('hidden');
    setStatus('Disconnected');
};

// Check WebXR availability on load
window.addEventListener('load', async () => {
    if (!navigator.xr) {
        setStatus('WebXR not available');
        connectBtn.disabled = true;
        return;
    }

    try {
        // Check for AR (passthrough) or VR support
        const arSupported = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
        const vrSupported = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);

        if (!arSupported && !vrSupported) {
            setStatus('VR/AR not supported');
            connectBtn.disabled = true;
        }
    } catch (error) {
        console.error('WebXR check failed:', error);
    }
});
