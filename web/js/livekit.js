// LiveKit transport — operator side. Drop-in alternative to setupWebRTC for
// sessions whose robot connected with transport="livekit". Produces the same
// state.* surface (state.cmdChannel / state.stateChannel shims, robot-cam video)
// the rest of the app already drives, so keyboard.js / vr.js / hud.js / send()
// and clock-sync work unchanged.

import { api } from './api.js';
import { ensureRobotCam, setStatus } from './dom.js';
import { state } from './state.js';
import { startClockSync, handleStateMessage, timeout } from './webrtc.js';

const CMD_TOPIC = 'cmd_unreliable';
const STATE_TOPIC = 'state_reliable';
const STATE_BACK_TOPIC = 'state_reliable_back';
const CONNECT_TIMEOUT_MS = 20000;

// Reused per packet on the hot data path — don't allocate per message.
const DEC = new TextDecoder();
const ENC = new TextEncoder();

function toU8(bytes) {
    if (bytes instanceof Uint8Array) return bytes;
    if (bytes instanceof ArrayBuffer) return new Uint8Array(bytes);
    if (ArrayBuffer.isView(bytes)) return new Uint8Array(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    return new Uint8Array(bytes);
}

export async function setupLiveKit(sessionId) {
    const LK = window.LivekitClient;
    if (!LK) throw new Error('LiveKit client SDK not loaded');
    setStatus('Connecting (LiveKit)...');

    // The broker mints a room-scoped token; LiveKit handles ICE/TURN itself, so
    // there is no SDP exchange or turn-credentials fetch here (unlike WebRTC).
    const data = await api('POST', `/sessions/${sessionId}/join`, { role: 'operator' });
    if (!data.url || !data.token) throw new Error('Broker did not return LiveKit url/token');

    // adaptiveStream pauses/downgrades subscribed video whose attached element
    // is hidden or zero-size — exactly the VR GL-texture <video> (display:none /
    // offscreen). Teleop wants full-rate always, so keep it off.
    const room = new LK.Room({ adaptiveStream: false, dynacast: true });
    state.room = room;

    // Robot camera track → the shared <video> element (same one the WebRTC path
    // feeds via pc.ontrack), so keyboard view + VR GL texture pick it up as-is.
    room.on(LK.RoomEvent.TrackSubscribed, (track) => {
        if (track.kind !== 'video') return;
        const existed = !!document.getElementById('robot-cam');
        const v = ensureRobotCam();
        track.attach(v);
        if (existed) v.style.display = 'block';
        v.play?.().catch(() => {});
    });

    // Robot → operator messages arrive on the back channel by protocol (the
    // robot always replies on state_reliable_back); LiveKit never echoes our own
    // published data, so the forward topic carries nothing inbound here.
    room.on(LK.RoomEvent.DataReceived, (payload, _participant, _kind, topic) => {
        if (topic === STATE_BACK_TOPIC) {
            handleStateMessage(DEC.decode(payload));
        }
    });

    room.on(LK.RoomEvent.Disconnected, () => console.info('[livekit] room disconnected'));

    // On connect failure/timeout, tear down the half-open Room — otherwise it
    // keeps reconnecting in the background and a retry spins up a second Room.
    try {
        await Promise.race([
            room.connect(data.url, data.token),
            timeout(CONNECT_TIMEOUT_MS, 'Timed out connecting to LiveKit'),
        ]);
    } catch (err) {
        await room.disconnect().catch(() => {});
        if (state.room === room) state.room = null;
        throw err;
    }

    // Shim the two outbound DataChannels onto LiveKit topics. send() (webrtc.js)
    // and startClockSync only touch .readyState + .send(), so these stand in
    // transparently; .close() is a no-op (room teardown happens in disconnect).
    // readyState tracks room.state so callers stop sending during a reconnect
    // (otherwise they'd fire into a half-open room); publishData is fire-and-
    // forget, so swallow its rejection to avoid unhandled promise rejections.
    const lp = room.localParticipant;
    const publish = (bytes, opts) => lp.publishData(bytes, opts).catch(() => {});
    const isOpen = () => (room.state === 'connected' ? 'open' : 'closed');
    state.cmdChannel = {
        get readyState() { return isOpen(); },
        send: (bytes) => publish(toU8(bytes), { reliable: false, topic: CMD_TOPIC }),
        close: () => {},
    };
    state.stateChannel = {
        get readyState() { return isOpen(); },
        send: (txt) => publish(ENC.encode(txt), { reliable: true, topic: STATE_TOPIC }),
        close: () => {},
    };

    startClockSync(state.stateChannel);
    // Video-stats reporter is skipped on LiveKit: it samples state.pc.getStats(),
    // which the SDK owns internally. (HUD video health is a follow-up.)
}
