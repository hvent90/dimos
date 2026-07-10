// Immersive WebXR cockpit for the xArm (manipulation). Parallel to vr.js (the
// Go2 drive cockpit): same generic shell — XR session, renderer, camera video
// panel, controller rays, hover, stats — but the input plane streams 6-DoF
// controller poses + gripper instead of thumbstick drive twists.
//
//   controller gripSpace pose ──cmd_unreliable──▶ PoseStamped (frame_id=hand)
//   trigger/buttons           ──cmd_unreliable──▶ Joy  (axes[2]=gripper, btn4=engage)
//
// Engage = hold the primary button (X left / A right); the robot recaptures the
// baseline pose on engage and streams deltas, so we send ABSOLUTE poses. The
// robot's webxr_to_robot owns the frame conversion — we send raw WebXR poses.

import * as THREE from 'three';

import { sampleCmdHz } from './hud.js';
import { createStallGate, videoMediaTime } from './stall.js';
import { disconnect } from './disconnect.js';
import { sendInterval, state } from './state.js';
import { buildArmCockpit, aui, onCmdAck, onRobotState } from './vrarmui.js';
import { getVRRenderer } from './vrrenderer.js';
import { buildEEFTwist, buildJoy, buildPoseStamped, sendCameraSelect, sendEstop } from './xarmcmd.js';

const HEAD = new THREE.Vector3(0, 1.55, 0);
// Two side-by-side camera screens (cam1 left, cam2 right), each ~16:9. The robot
// muxes both cams into ONE hstacked 1696×480 video track; we split that texture
// down the middle so each panel shows one full-aspect feed — a dual-monitor
// cockpit rather than a single switched view.
const SCREEN = { w: 1.24, h: 0.70, y: 1.52, z: -1.62 };
const SCREEN_GAP = 0.06;     // seam between the two panels (metres)
const SCREEN_YAW = 0.20;     // toe-in so both face the operator (~11°)

let renderer = null, scene = null, camera = null;
let cockpit = null, controllers = [];
// videoMeshes[0] = left/cam1 (texture left half), [1] = right/cam2 (right half).
let videoMeshes = [], videoTexes = [];
let stallGate = null;
let camEl = null;
let estopNonce = 0;
let _estopCooldown = 0;
const raycaster = new THREE.Raycaster();
const _rayOrigin = new THREE.Vector3();
const _rayDir = new THREE.Vector3();

function buildScene() {
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(70, 1, 0.05, 100);

    // Two panels, toed-in toward the operator. Left edge of the right panel and
    // right edge of the left panel meet near centre with a small gap.
    const halfW = SCREEN.w / 2;
    const centreX = halfW + SCREEN_GAP / 2;
    videoMeshes = [];
    for (let i = 0; i < 2; i++) {
        const mesh = new THREE.Mesh(
            new THREE.PlaneGeometry(SCREEN.w, SCREEN.h),
            new THREE.MeshBasicMaterial({ color: 0x0d0e0e }),
        );
        const sign = i === 0 ? -1 : 1;  // left panel, then right panel
        mesh.position.set(sign * centreX, SCREEN.y, SCREEN.z);
        mesh.rotation.y = -sign * SCREEN_YAW;  // toe-in
        mesh.renderOrder = 1;
        scene.add(mesh);
        videoMeshes.push(mesh);
    }

    cockpit = buildArmCockpit(scene, HEAD);
}

function initControllers() {
    const lineGeo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -5),
    ]);
    for (let i = 0; i < 2; i++) {
        const ctrl = renderer.xr.getController(i);
        const laser = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0xb0e1f0, transparent: true, opacity: 0.5 }));
        laser.scale.z = 1;
        ctrl.add(laser);
        const dot = new THREE.Mesh(
            new THREE.SphereGeometry(0.012, 12, 12),
            new THREE.MeshBasicMaterial({ color: 0xb0e1f0 }),
        );
        dot.visible = false;
        ctrl.userData.dot = dot;
        scene.add(dot);
        ctrl.addEventListener('selectstart', () => onSelect(ctrl));
        scene.add(ctrl);
        controllers.push(ctrl);
    }
}

function onSelect(ctrl) {
    const hit = raycastPanels(ctrl);
    if (hit) cockpit.onClick(hit.object.userData.panel, hit.uv);
}

function raycastPanels(ctrl) {
    _rayOrigin.setFromMatrixPosition(ctrl.matrixWorld);
    _rayDir.set(0, 0, -1).applyQuaternion(ctrl.quaternion).normalize();
    raycaster.set(_rayOrigin, _rayDir);
    const hits = raycaster.intersectObjects(cockpit.meshes, false);
    return hits.length ? hits[0] : null;
}

function updateVideoTexture() {
    const v = camEl;
    if (!v || v.readyState < 2 || !v.videoWidth) return;

    // (Re)bind a texture per panel off the same video element. Each panel samples
    // one horizontal half of the muxed frame: left→cam1, right→cam2.
    if (videoTexes.length !== 2 || videoTexes[0].image !== v) {
        for (const t of videoTexes) t.dispose();
        videoTexes = videoMeshes.map((mesh) => {
            const tex = new THREE.VideoTexture(v);
            tex.colorSpace = THREE.SRGBColorSpace;
            mesh.material.dispose();
            mesh.material = new THREE.MeshBasicMaterial({ map: tex });
            return tex;
        });
    }

    // Vertical crop: drop the latency stamp strip (rows appended below the frame).
    const strip = state.liveStats.stampStripPx || 0;
    const yFrac = strip && v.videoHeight ? strip / v.videoHeight : 0;

    // Horizontal split. Two cams → wide frame (aspect ≥ ~2.4): each panel takes
    // its half. One cam → square-ish frame: both panels show the whole frame
    // (graceful fallback until the robot sends both).
    const dual = v.videoWidth >= v.videoHeight * 2;
    for (let i = 0; i < videoTexes.length; i++) {
        const tex = videoTexes[i];
        tex.offset.set(dual ? i * 0.5 : 0, yFrac);
        tex.repeat.set(dual ? 0.5 : 1, 1 - yFrac);
    }
}

// ── Arm command plane: stream controller pose + Joy per hand ─────────
let lastSend = 0;

function streamArmPose(frame) {
    const now = performance.now();
    if (now - lastSend < sendInterval) return;
    lastSend = now;

    const chan = state.cmdChannel;
    if (!chan || chan.readyState !== 'open') return;

    // Video-freshness gate: don't stream poses onto a frozen frame — the
    // operator would be commanding blind. (No held-state to track: any pose is
    // only acted on while a hand is engaged, and engage needs a live view.)
    const gate = stallGate.sample(videoMediaTime(camEl), now, false);
    state.videoStall = gate;
    if (gate.blocked) return;

    const nowMs = Date.now() + state.clockOffsetMs;
    // Menu button (index 6, either hand) → E-STOP in both modes.
    for (const src of frame.session.inputSources) {
        if (src.gamepad?.buttons[6]?.pressed) { triggerEstop(); break; }
    }
    if (aui.controlMode === 'joystick') {
        streamArmJoystick(frame, chan, nowMs);
    } else {
        streamArmPoseMode(frame, chan, nowMs);
    }
}

// Pose mode (default): stream absolute controller pose + Joy per hand. The robot
// re-bases on engage and applies deltas → TeleopIKTask.
function streamArmPoseMode(frame, chan, nowMs) {
    for (const src of frame.session.inputSources) {
        const space = src.gripSpace || src.targetRaySpace;
        if (!space) continue;
        const hand = src.handedness;
        if (hand !== 'left' && hand !== 'right') continue;
        const pose = frame.getPose(space, state.xrRefSpace);
        if (!pose) continue;

        chan.send(buildPoseStamped(hand, pose.transform.position, pose.transform.orientation, nowMs).encode());
        state.cmdSendCount++;

        if (src.gamepad) chan.send(buildJoy(hand, src.gamepad, nowMs).encode());
    }
}

// Joystick mode: thumbsticks drive an EE-twist (coordinator's eef_twist task,
// same plane as the browser keyboard cockpit). No trigger → translate; trigger
// held → rotate. Right stick = X/Y, left stick = Z / yaw. Tuned conservatively;
// refine axis assignment on hardware.
const VR_LIN_SPEED = 0.15;   // m/s at full stick
const VR_ANG_SPEED = 0.9;    // rad/s at full stick
const STICK_DEADZONE = 0.12;

function dz(v) { return Math.abs(v) < STICK_DEADZONE ? 0 : v; }

function stickXY(gp) {
    // xr-standard exposes the thumbstick on axes[2]/[3]; older mappings on [0]/[1].
    const xr = gp.mapping === 'xr-standard';
    return [dz((xr ? gp.axes[2] : gp.axes[0]) ?? 0), dz((xr ? gp.axes[3] : gp.axes[1]) ?? 0)];
}

function streamArmJoystick(frame, chan, nowMs) {
    let left = null, right = null;
    let rotate = false;
    for (const src of frame.session.inputSources) {
        if (!src.gamepad) continue;
        if (src.handedness === 'left') left = src.gamepad;
        else if (src.handedness === 'right') right = src.gamepad;
        // Trigger (button 0) on either hand → rotate mode.
        if (src.gamepad.buttons[0]?.value > 0.5) rotate = true;
    }
    const linear = { x: 0, y: 0, z: 0 };
    const angular = { x: 0, y: 0, z: 0 };

    if (right) {
        const [rx, ry] = stickXY(right);
        if (rotate) { angular.x += ry * VR_ANG_SPEED; angular.y += rx * VR_ANG_SPEED; }  // roll / pitch
        else { linear.x += -ry * VR_LIN_SPEED; linear.y += rx * VR_LIN_SPEED; }          // fwd/back · left/right
    }
    if (left) {
        const [lx, ly] = stickXY(left);
        if (rotate) { angular.z += lx * VR_ANG_SPEED; }         // yaw
        else { linear.z += -ly * VR_LIN_SPEED; }                // up/down
    }

    chan.send(buildEEFTwist(linear, angular, nowMs).encode());
    state.cmdSendCount++;
}

function triggerEstop() {
    const now = performance.now();
    if (now < _estopCooldown || aui.estopped) return;
    _estopCooldown = now + 1500;
    aui.estopped = true;
    sendEstop(state.stateChannel, () => ++estopNonce);
    cockpit.panels.forEach((p) => p.markDirty());
}

let lastCmdSampleMs = 0;
function tickCmdHz(nowMs) {
    if (!lastCmdSampleMs) { lastCmdSampleMs = nowMs; return; }
    if (nowMs - lastCmdSampleMs < 1000) return;
    sampleCmdHz((nowMs - lastCmdSampleMs) / 1000);
    lastCmdSampleMs = nowMs;
}

function updateHover() {
    for (const p of cockpit.panels) p.setHover(null);
    for (const ctrl of controllers) {
        const dot = ctrl.userData.dot;
        const hit = raycastPanels(ctrl);
        if (!hit) { dot.visible = false; continue; }
        dot.visible = true;
        dot.position.copy(hit.point);
        const panel = hit.object.userData.panel;
        panel.setHover(panel.hitTest(hit.uv));
    }
}

// Ask the robot for BOTH cameras once the state channel opens (the mux defaults
// to cam1 only). One-shot: the dual-screen cockpit always wants both muxed.
let _requestedBothCams = false;
function requestBothCams() {
    if (_requestedBothCams) return;
    if (!state.stateChannel || state.stateChannel.readyState !== 'open') return;
    sendCameraSelect(state.stateChannel, ['cam1', 'cam2']);
    _requestedBothCams = true;
}

function onFrame(timeMs, frame) {
    if (!camEl) camEl = document.getElementById('robot-cam');
    requestBothCams();
    if (frame) streamArmPose(frame);
    updateHover();
    updateVideoTexture();
    // Tint both screens red when the robot video is stalled/frozen.
    const stalled = state.videoStall?.stalled;
    for (const mesh of videoMeshes) {
        if (mesh.material.map) mesh.material.color.setHex(stalled ? 0x552222 : 0xffffff);
    }
    cockpit.tick(timeMs);
    tickCmdHz(timeMs);
    renderer.render(scene, camera);
}

export async function startArmVR() {
    lastCmdSampleMs = 0; lastSend = 0; _requestedBothCams = false;
    stallGate = createStallGate();
    state.videoStall = { stalled: false, blocked: false, armed: false };
    document.getElementById('canvas').style.display = 'block';
    renderer = getVRRenderer();  // one shared renderer across all VR cockpits
    buildScene();
    controllers = [];

    // Passthrough (AR) when available; opaque VR otherwise. Must run inside the
    // Connect click gesture.
    let session = null, ar = false;
    try {
        session = await navigator.xr.requestSession('immersive-ar', {
            requiredFeatures: ['local-floor'], optionalFeatures: ['hand-tracking'],
        });
        ar = true;
    } catch (e) {
        session = await navigator.xr.requestSession('immersive-vr', {
            requiredFeatures: ['local-floor'], optionalFeatures: ['hand-tracking'],
        });
    }
    scene.background = ar ? null : new THREE.Color(0x0a0b0b);
    renderer.setClearAlpha(ar ? 0 : 1);

    state.xrSession = session;
    await renderer.xr.setSession(session);
    // Sample controller poses against a reference space we own (matches the
    // working quest reference client). three.js's getReferenceSpace() can lag a
    // recenter/reset, which would freeze getPose and stall the arm.
    state.xrRefSpace = await session.requestReferenceSpace('local-floor');
    initControllers();

    // Route acks + robot-state onto the arm cockpit.
    state.onCmdAck = onCmdAck;
    state.onRobotState = onRobotState;

    session.addEventListener('end', () => {
        state.xrSession = null;
        state.onCmdAck = state.onRobotState = null;
        renderer.setAnimationLoop(null);
        cockpit?.dispose();
        cockpit = null;
        for (const t of videoTexes) t.dispose();
        for (const m of videoMeshes) { m.geometry.dispose(); m.material.dispose(); }
        videoTexes = []; videoMeshes = [];
        camEl = null;
        disconnect();
    });
    renderer.setAnimationLoop(onFrame);
}
