// VR cockpit preview — no broker, no robot. Open #vrpreview on a headset,
// press the button (requestSession needs the gesture), and the full VR
// cockpit renders against faked data:
//   - fake state/cmd channels that echo cmd_acks like an obedient robot
//   - synthesized occupancy-grid map + robot marker driving a lap
//   - canvas test-pattern piped into #robot-cam via captureStream()
//   - telemetry (fps/rtt/cmd/battery) wiggling at plausible values
// Everything is local: nothing touches the broker or a real robot.

import { ensureRobotCam } from './dom.js';
import { state } from './state.js';
import { startVR } from './vr.js';

const timers = [];

// ── fake robot: echo acks + track state like the real firmware would ──
const robot = {
    posture: 'StandReady', estopped: false, obstacle: true,
    light: 0, cams: ['cam1'], rage: false,
};

function fakeChannels() {
    state.cmdChannel = { readyState: 'open', send: () => { state.cmdSendCount++; } };
    state.stateChannel = {
        readyState: 'open',
        send: (txt) => {
            let m; try { m = JSON.parse(txt); } catch (_) { return; }
            // Update the fake robot the way the real one would.
            if (m.type === 'sport_cmd') {
                if (['StandReady', 'StandDown', 'Sit', 'RecoveryStand', 'Damp'].includes(m.name)) robot.posture = m.name;
            } else if (m.type === 'estop') { robot.estopped = true; robot.posture = 'Damp'; }
            else if (m.type === 'estop_clear') robot.estopped = false;
            else if (m.type === 'obstacle_avoidance') robot.obstacle = !!m.enabled;
            else if (m.type === 'light') robot.light = m.brightness ?? 0;
            else if (m.type === 'camera_select') robot.cams = m.cams || ['cam1'];
            else if (m.type === 'set_mode') robot.rage = m.mode === 'rage';
            // Ack anything nonce'd, like the robot's command executor.
            if (m.nonce != null) {
                const delay = m.name === 'StandReady' ? 1200 : 250;
                timers.push(setTimeout(() => state.onCmdAck?.({ type: 'cmd_ack', nonce: m.nonce, ok: true }), delay));
            }
        },
    };
}

// ── fake occupancy grid: rooms + corridor, robot lapping the space ────
function fakeMap() {
    const W = 160, H = 120, RES = 0.05;              // 8m × 6m at 5cm cells
    const c = document.createElement('canvas');
    c.width = W; c.height = H;
    const x = c.getContext('2d');
    x.fillStyle = '#c8cfcf'; x.fillRect(0, 0, W, H);          // free space
    x.fillStyle = '#1a1d1d';                                   // walls
    x.fillRect(0, 0, W, 3); x.fillRect(0, H - 3, W, 3);
    x.fillRect(0, 0, 3, H); x.fillRect(W - 3, 0, 3, H);
    x.fillRect(50, 0, 4, 70);                                  // room divider
    x.fillRect(100, 50, 4, 70);
    x.fillRect(50, 66, 30, 4);
    x.fillStyle = '#5a6363';                                   // unknown blobs
    x.fillRect(120, 15, 18, 12); x.fillRect(20, 90, 14, 10);
    const png_b64 = c.toDataURL('image/png').split(',')[1];
    state.onMap?.({ type: 'map', png_b64, w: W, h: H, res: RES, origin: [-4.0, -3.0] });

    // Robot lapping a rounded rectangle inside the free space.
    let t = 0;
    timers.push(setInterval(() => {
        t += 0.02;
        const px = 1.6 * Math.cos(t), py = 1.1 * Math.sin(t);
        state.onOdom?.({ x: px, y: py, yaw: t + Math.PI / 2, ts: Date.now() / 1000 });
    }, 100));
}

// ── fake camera: animated test pattern → #robot-cam ──────────────────
function fakeVideo() {
    const c = document.createElement('canvas');
    c.width = 640; c.height = 360;
    const x = c.getContext('2d');
    let t = 0;
    timers.push(setInterval(() => {
        t += 1;
        const g = x.createLinearGradient(0, 0, 640, 360);
        g.addColorStop(0, `hsl(${(t * 2) % 360},35%,18%)`);
        g.addColorStop(1, '#0d0e0e');
        x.fillStyle = g; x.fillRect(0, 0, 640, 360);
        x.strokeStyle = '#b0e1f0'; x.lineWidth = 2;
        x.strokeRect(20, 20, 600, 320);
        x.fillStyle = '#b0e1f0'; x.font = '600 28px monospace';
        x.fillText('VR PREVIEW — no robot', 40, 60);
        x.font = '20px monospace';
        x.fillText(new Date().toLocaleTimeString(), 40, 95);
        const cx = 320 + 180 * Math.cos(t / 20), cy = 200 + 90 * Math.sin(t / 20);
        x.beginPath(); x.arc(cx, cy, 14, 0, Math.PI * 2); x.fill();
    }, 66));
    const v = ensureRobotCam();
    v.srcObject = c.captureStream(15);
    v.play?.().catch(() => {});
}

// ── fake telemetry: stats panel food ─────────────────────────────────
function fakeTelemetry() {
    let soc = 87;
    timers.push(setInterval(() => {
        const w = (n, a) => +(n + (Math.random() - 0.5) * a).toFixed(1);
        state.liveStats.rttMs = w(46, 8);
        state.liveStats.video = {
            fps: w(29, 3), kbps: w(2400, 400), width: 640, height: 360,
            loss_pct: Math.max(0, w(0.4, 0.6)), jitter_ms: w(9, 4),
            jitter_buffer_ms: w(42, 10), decode_ms: w(6, 3),
            e2e_latency_ms: w(180, 30), freezes: 0, frames_dropped: 3,
        };
        state.liveStats.cmd = { latency_ms: w(48, 10), jitter_ms: w(7, 3), rate_hz: w(78, 4), throughput_bps: 41000 };
        soc = Math.max(5, soc - 0.02);
        state.liveStats.soc = Math.round(soc);
        state.liveStats.iceType = 'stun';
        state.onRobotState?.({
            posture: robot.posture, rage: robot.rage, obstacle_avoidance: robot.obstacle,
            light: robot.light, cams: robot.cams, estopped: robot.estopped, video_stalled: false,
        });
    }, 1000));
}

export function renderVRPreview(c) {
    c.innerHTML = `
    <div class="min-h-screen flex flex-col items-center justify-center p-6 text-center fade-in">
        <span class="crt-glow text-dim-500 font-bold tracking-widest text-2xl mb-2">DIMENSIONAL</span>
        <p class="term-caps text-gray-500 text-xs mb-8">// VR cockpit preview — no robot, faked data</p>
        <button id="enter-vr" class="px-10 py-6 bg-dim-500 hover:bg-dim-600 text-bg-950 text-xl font-bold rounded-2xl">
            ENTER VR PREVIEW
        </button>
        <p id="vr-preview-status" class="text-gray-500 text-sm mt-6 max-w-md">
            Requires a WebXR headset browser (Quest). Panels: map left ·
            camera front · buttons right · stats far right. Buttons ack
            locally; nothing is sent anywhere.
        </p>
    </div>`;
    document.getElementById('enter-vr').addEventListener('click', async () => {
        const status = document.getElementById('vr-preview-status');
        if (!navigator.xr) { status.textContent = 'WebXR not available in this browser.'; return; }
        try {
            state.activeRobot = { session_id: 'preview', robot_name: 'vr-preview', transport: 'cloudflare' };
            state.driveEnabled = true;
            await startVR();          // registers onCmdAck/onRobotState/onMap/onOdom
            fakeChannels();
            fakeMap();
            fakeVideo();
            fakeTelemetry();
        } catch (e) {
            status.textContent = 'VR failed: ' + e.message;
        }
    });
}

// Best-effort cleanup if the page navigates away mid-preview.
window.addEventListener('pagehide', () => timers.forEach((t) => { clearTimeout(t); clearInterval(t); }));
