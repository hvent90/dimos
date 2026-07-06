// VR cockpit UI — the Go2 cockpit (go2.js) ported to spatial panels.
//
// Each panel is a 2D canvas textured onto a three.js plane; interaction is
// controller-ray raycast → panel UV → canvas px → hit-test against the panel's
// registered rectangles. That reuses the cockpit's visual language (rounded
// chips, brand palette, health tints) and its command protocol verbatim —
// only the input model changes (ray-click instead of mouse, thumbstick drive).
//
// Layout (world-anchored in local-floor, user starts at origin looking -Z):
//   MAP left · CAMERA front-centre · BUTTONS right · STATS far-right column.
//
// Commands mirror views/go2.js exactly (same JSON on state.stateChannel), so
// the robot side needs no changes and telemetry reconcile / acks flow back
// through the same state.onRobotState / state.onCmdAck hooks.

import * as THREE from 'three';

import { hudDetailRows, healthColor, statsHealth, transportLabel } from './hud.js';
import { state } from './state.js';

// Palette (matches index.html cockpit CSS + hud.js).
const C = {
    bg: 'rgba(18,19,19,0.92)', bgSolid: '#121313', panel: '#0d0e0e',
    line: '#2a2a2a', text: '#d1d5db', dim: '#6b7280', cyan: '#b0e1f0',
    cyanDim: '#3d6a7a', good: '#34d399', warn: '#ffcc00', bad: '#ff5252',
    estopBg: '#7a1f1f', estopBorder: '#d97777',
};
const HEALTH = { good: C.good, warn: C.warn, bad: C.bad };

// ── Command catalog (verbatim from go2.js) ───────────────────────────
const POSTURE = [
    { name: 'StandReady', label: 'Stand / Drive' },
    { name: 'StandDown', label: 'Sit' },
];
const ACTIONS = [
    { name: 'Hello', label: 'Shake Hand' },
    { name: 'Stretch', label: 'Stretch' },
    { name: 'FrontPounce', label: 'Pounce' },
    { name: 'FrontJump', label: 'Jump Fwd' },
];
const SPEEDS = [
    { mode: 'normal', label: 'Normal', scale: { lin: 0.5, ang: 0.5 } },
    { mode: 'high', label: 'High', scale: { lin: 1.0, ang: 1.0 } },
    { mode: 'rage', label: 'Rage', scale: { lin: 2.0, ang: 1.5 } },
];
const CAMS = [{ id: 'cam1', label: 'Cam 1' }, { id: 'cam2', label: 'Cam 2' }];
// Discrete light presets — a raycast slider is fiddly; presets read cleanly.
const LIGHTS = [
    { label: 'Off', v: 0 }, { label: 'Low', v: 0.34 },
    { label: 'Med', v: 0.67 }, { label: 'Full', v: 1.0 },
];
const CONFIRM_ACTIONS = new Set(['FrontPounce', 'FrontJump']);
const POSTURE_STATE = { StandReady: 'StandReady', StandDown: 'StandDown', RecoveryStand: 'RecoveryStand', Sit: 'Sit' };
const GO2_LEN_M = 0.70, GO2_WID_M = 0.31;  // footprint, matches cockpit glyph

// ── Shared cockpit UI state (mirror of go2.js `ui`) ──────────────────
export const vui = {
    posture: 'StandReady', estopped: false, speedMode: 'normal',
    selectedCams: ['cam1'], obstacleAvoid: true, light: 0,
    robotVideoStalled: false, nonce: 0,
    pending: new Map(),        // nonce → { region, panel, timer }
    confirm: null,             // { name, expiry } two-press guard for acrobatics
    lastMap: null, lastOdom: null, navGoal: null,
};

function chanReady() {
    return state.stateChannel && state.stateChannel.readyState === 'open';
}
function sendJSON(obj) { if (chanReady()) state.stateChannel.send(JSON.stringify(obj)); }

// ── Panel: canvas → CanvasTexture → plane, with hit regions ──────────
class Panel {
    constructor({ wM, hM, cw, ch, opacity = 1 }) {
        this.cw = cw; this.ch = ch;
        this.canvas = document.createElement('canvas');
        this.canvas.width = cw; this.canvas.height = ch;
        this.ctx = this.canvas.getContext('2d');
        this.tex = new THREE.CanvasTexture(this.canvas);
        this.tex.colorSpace = THREE.SRGBColorSpace;
        this.mesh = new THREE.Mesh(
            new THREE.PlaneGeometry(wM, hM),
            new THREE.MeshBasicMaterial({ map: this.tex, transparent: true, opacity }),
        );
        this.mesh.userData.panel = this;
        this.regions = [];     // { id, x, y, w, h }
        this.hoverId = null;
        this.dirty = true;
    }
    place(pos, headPos) {
        this.mesh.position.copy(pos);
        this.mesh.lookAt(headPos);  // plane front (+Z) faces the user
    }
    markDirty() { this.dirty = true; }
    // UV (three) → canvas px. VideoTexture/CanvasTexture are flipY, so v=0 is
    // the bottom row; canvas y grows down → row = (1-v)*ch.
    hitTest(uv) {
        const px = uv.x * this.cw, py = (1 - uv.y) * this.ch;
        for (const r of this.regions) {
            if (px >= r.x && px <= r.x + r.w && py >= r.y && py <= r.y + r.h) return r.id;
        }
        return null;
    }
    setHover(id) { if (id !== this.hoverId) { this.hoverId = id; this.dirty = true; } }

    // ── drawing primitives ──
    bg() {
        const x = this.ctx;
        x.clearRect(0, 0, this.cw, this.ch);
        x.fillStyle = C.bg;
        roundRect(x, 2, 2, this.cw - 4, this.ch - 4, 18); x.fill();
        x.strokeStyle = C.line; x.lineWidth = 2; x.stroke();
        this.regions = [];
    }
    header(text) {
        const x = this.ctx;
        x.fillStyle = C.dim;
        x.font = '600 22px ui-monospace, monospace';
        x.fillText(text.toUpperCase(), 24, 40);
    }
    // A chip button; `st` ∈ idle|active|pending|done|error|confirm.
    chip(id, bx, by, bw, bh, label, st = 'idle') {
        const x = this.ctx, hot = this.hoverId === id;
        let fill = C.bgSolid, border = C.line, txt = C.text;
        if (st === 'active') { fill = C.cyan; border = C.cyan; txt = C.panel; }
        else if (st === 'done') { fill = C.cyan; border = C.cyan; txt = C.panel; }
        else if (st === 'error') { fill = '#4a1d1d'; border = C.estopBorder; txt = '#f3b4b4'; }
        else if (st === 'pending') { border = C.cyanDim; txt = C.dim; }
        else if (st === 'confirm') { fill = '#4a3a1d'; border = C.warn; txt = C.warn; }
        if (hot && st === 'idle') border = C.cyan;
        x.fillStyle = fill; roundRect(x, bx, by, bw, bh, 10); x.fill();
        x.lineWidth = hot ? 2.5 : 1.5; x.strokeStyle = border; x.stroke();
        x.fillStyle = txt;
        x.font = '600 20px ui-monospace, monospace';
        x.textAlign = 'center'; x.textBaseline = 'middle';
        x.fillText(label, bx + bw / 2, by + bh / 2 + 1);
        x.textAlign = 'left'; x.textBaseline = 'alphabetic';
        this.regions.push({ id, x: bx, y: by, w: bw, h: bh });
    }
    dispose() {
        this.tex.dispose();
        this.mesh.geometry.dispose();
        this.mesh.material.dispose();
    }
}

function roundRect(x, bx, by, bw, bh, r) {
    x.beginPath();
    x.moveTo(bx + r, by);
    x.arcTo(bx + bw, by, bx + bw, by + bh, r);
    x.arcTo(bx + bw, by + bh, bx, by + bh, r);
    x.arcTo(bx, by + bh, bx, by, r);
    x.arcTo(bx, by, bx + bw, by, r);
    x.closePath();
}

// ── Command senders (mirror go2.js; add ack tracking per region) ─────
function nonceCmd(panel, region, obj, timeoutMs = 3000) {
    const nonce = ++vui.nonce;
    obj.nonce = nonce;
    sendJSON(obj);
    setRegionState(panel, region, 'pending');
    const timer = setTimeout(() => resolveAck(nonce, false), timeoutMs);
    vui.pending.set(nonce, { panel, region, timer });
    return nonce;
}

function sendSportCmd(name, panel, region) {
    if (!chanReady() || vui.estopped) return;
    if (CONFIRM_ACTIONS.has(name)) {
        // Two-press confirm (no confirm() dialog in XR): first press arms.
        const now = performance.now();
        if (!vui.confirm || vui.confirm.name !== name || now > vui.confirm.expiry) {
            vui.confirm = { name, expiry: now + 4000 };
            panel.markDirty();
            return;
        }
        vui.confirm = null;
    }
    nonceCmd(panel, region, { type: 'sport_cmd', name },
        name === 'StandReady' ? 9000 : 3000);
}

function selectSpeed(mode, send = true) {
    const spec = SPEEDS.find((s) => s.mode === mode);
    if (!spec) return;
    vui.speedMode = mode;
    state.speedScale = spec.scale;
    if (send) sendJSON({ type: 'set_mode', mode, nonce: ++vui.nonce });
}

function toggleCam(id, panel) {
    const sel = new Set(vui.selectedCams);
    if (sel.has(id)) { if (sel.size === 1) return; sel.delete(id); } else sel.add(id);
    vui.selectedCams = CAMS.map((c) => c.id).filter((cid) => sel.has(cid));
    sendJSON({ type: 'camera_select', cams: vui.selectedCams });
    panel.markDirty();
}

function toggleObstacle(panel, region) {
    if (!chanReady()) return;
    vui.obstacleAvoid = !vui.obstacleAvoid;
    nonceCmd(panel, region, { type: 'obstacle_avoidance', enabled: vui.obstacleAvoid });
}

function setLight(v, panel, region) {
    if (!chanReady()) return;
    vui.light = v;
    nonceCmd(panel, region, { type: 'light', brightness: v });
}

function estop(panel) {
    vui.estopped = true;
    sendJSON({ type: 'estop', nonce: ++vui.nonce });
    sendJSON({ type: 'sport_cmd', name: 'Damp', nonce: ++vui.nonce });  // legacy fallback
    vui.posture = 'Damp';
    panel.markDirty();
}
function rearm(panel) {
    vui.estopped = false;
    sendJSON({ type: 'estop_clear', nonce: ++vui.nonce });
    panel.markDirty();
}

// ── Ack + robot-state reconcile (registered on state.* by vr.js) ─────
function setRegionState(panel, region, st) {
    panel._rstate = panel._rstate || {};
    panel._rstate[region] = st;
    panel.markDirty();
}
function resolveAck(nonce, ok) {
    const p = vui.pending.get(nonce);
    if (!p) return;
    clearTimeout(p.timer);
    vui.pending.delete(nonce);
    setRegionState(p.panel, p.region, ok ? 'done' : 'error');
    setTimeout(() => { setRegionState(p.panel, p.region, 'idle'); }, 700);
}
export function onCmdAck(msg) { resolveAck(msg.nonce, !!msg.ok); }

export function onRobotState(s) {
    if (vui.pending.size > 0) return;  // don't fight a command mid-flight
    if (typeof s.posture === 'string' && POSTURE_STATE[s.posture]) vui.posture = s.posture;
    if (typeof s.estopped === 'boolean') vui.estopped = s.estopped;
    if (typeof s.obstacle_avoidance === 'boolean') vui.obstacleAvoid = s.obstacle_avoidance;
    if (typeof s.light === 'number') vui.light = s.light;
    if (Array.isArray(s.cams) && s.cams.length) vui.selectedCams = s.cams.filter((c) => CAMS.some((k) => k.id === c));
    if (typeof s.rage === 'boolean') vui.speedMode = s.rage ? 'rage' : (vui.speedMode === 'rage' ? 'normal' : vui.speedMode);
    if (typeof s.video_stalled === 'boolean') vui.robotVideoStalled = s.video_stalled;
    // Drive gate mirrors the cockpit: only StandReady drives.
    state.driveEnabled = vui.posture === 'StandReady' && !vui.estopped;
    for (const pnl of _allPanels) pnl.markDirty();
}

// ── Minimap: decode the base64-PNG occupancy grid into a texture ─────
export function onMap(msg) {
    if (!msg || !msg.png_b64) return;
    const img = new Image();
    img.onload = () => { vui.lastMap = { ...msg, img }; _mapPanel?.markDirty(); };
    img.onerror = () => {};
    img.src = 'data:image/png;base64,' + msg.png_b64;
}
export function onOdom(msg) { if (msg) { vui.lastOdom = msg; _mapPanel?.markDirty(); } }

// ─────────────────────────────────────────────────────────────────────
// Panel renderers
// ─────────────────────────────────────────────────────────────────────

function renderButtons(p) {
    p.bg();
    const st = (region, active) => {
        const s = (p._rstate && p._rstate[region]) || 'idle';
        return s !== 'idle' ? s : (active ? 'active' : 'idle');
    };
    let y = 60;
    const W = p.cw, PAD = 24, GAP = 12;
    const row = (items, h, render) => {
        const bw = (W - 2 * PAD - GAP * (items.length - 1)) / items.length;
        items.forEach((it, i) => render(it, PAD + i * (bw + GAP), y, bw, h));
        y += h + 18;
    };
    const sect = (t) => { p.ctx.fillStyle = C.dim; p.ctx.font = '600 18px ui-monospace,monospace'; p.ctx.fillText(t.toUpperCase(), PAD, y - 6); y += 8; };

    sect('Posture');
    row(POSTURE, 56, (it, bx, by, bw, bh) => {
        const active = vui.posture === it.name || (it.name === 'StandReady' && vui.posture === 'StandReady');
        p.chip('sport:' + it.name, bx, by, bw, bh, it.label, st('sport:' + it.name, active));
    });
    sect('Actions');
    // 2×2
    for (let r = 0; r < 2; r++) {
        row(ACTIONS.slice(r * 2, r * 2 + 2), 50, (it, bx, by, bw, bh) => {
            const confirming = vui.confirm && vui.confirm.name === it.name && performance.now() < vui.confirm.expiry;
            const label = confirming ? 'Confirm?' : it.label;
            const s = confirming ? 'confirm' : st('sport:' + it.name, false);
            p.chip('sport:' + it.name, bx, by, bw, bh, label, s);
        });
    }
    sect('Speed');
    row(SPEEDS, 48, (it, bx, by, bw, bh) =>
        p.chip('speed:' + it.mode, bx, by, bw, bh, it.label, vui.speedMode === it.mode ? 'active' : 'idle'));
    sect('Cameras');
    row(CAMS, 46, (it, bx, by, bw, bh) =>
        p.chip('cam:' + it.id, bx, by, bw, bh, it.label, vui.selectedCams.includes(it.id) ? 'active' : 'idle'));
    sect('Obstacle avoidance');
    row([{}], 46, (it, bx, by, bw, bh) =>
        p.chip('obstacle', bx, by, bw, bh, vui.obstacleAvoid ? 'ON' : 'OFF', st('obstacle', vui.obstacleAvoid)));
    sect('Light');
    row(LIGHTS, 46, (it, bx, by, bw, bh) => {
        const active = Math.abs(vui.light - it.v) < 0.08;
        p.chip('light:' + it.v, bx, by, bw, bh, it.label, active ? 'active' : 'idle');
    });

    // E-STOP — big, always at the bottom; re-arm appears when latched.
    y = p.ch - (vui.estopped ? 130 : 74);
    const ex = PAD, ew = W - 2 * PAD;
    p.ctx.fillStyle = vui.estopped ? C.estopBorder : C.estopBg;
    roundRect(p.ctx, ex, y, ew, 58, 12); p.ctx.fill();
    p.ctx.lineWidth = 2.5; p.ctx.strokeStyle = vui.estopped ? '#fff' : C.estopBorder; p.ctx.stroke();
    p.ctx.fillStyle = '#fff'; p.ctx.font = '700 24px ui-monospace,monospace';
    p.ctx.textAlign = 'center'; p.ctx.textBaseline = 'middle';
    p.ctx.fillText(vui.estopped ? 'STOPPED' : '■ EMERGENCY STOP', ex + ew / 2, y + 30);
    p.ctx.textAlign = 'left'; p.ctx.textBaseline = 'alphabetic';
    p.regions.push({ id: 'estop', x: ex, y, w: ew, h: 58 });
    if (vui.estopped) {
        p.chip('rearm', ex, y + 66, ew, 44, 're-arm →', 'idle');
    }
}

function renderStats(p) {
    p.bg();
    const x = p.ctx;
    // Health dot + transport header.
    x.fillStyle = healthColor();
    x.beginPath(); x.arc(30, 34, 9, 0, Math.PI * 2); x.fill();
    x.fillStyle = C.text; x.font = '600 20px ui-monospace,monospace';
    x.fillText(transportLabel(), 48, 40);
    const soc = state.liveStats?.soc;
    x.fillStyle = soc == null ? C.dim : (soc > 40 ? C.cyan : soc > 15 ? C.warn : C.bad);
    x.textAlign = 'right'; x.fillText(soc == null ? '—%' : `${Math.round(soc)}%`, p.cw - 24, 40);
    x.textAlign = 'left';

    let y = 72;
    for (const g of hudDetailRows()) {
        x.fillStyle = C.dim; x.font = '600 15px ui-monospace,monospace';
        x.fillText(g.group.toUpperCase(), 24, y); y += 22;
        for (const r of g.rows) {
            x.fillStyle = C.dim; x.font = '15px ui-monospace,monospace';
            x.fillText(r.label, 30, y);
            x.fillStyle = HEALTH[r.health] || C.text;
            x.textAlign = 'right'; x.font = '600 15px ui-monospace,monospace';
            x.fillText(String(r.value), p.cw - 24, y);
            x.textAlign = 'left';
            y += 22;
        }
        y += 8;
    }
}

// Draw the decoded occupancy grid + robot glyph + nav goal. Mirrors go2.js
// drawMap (letterbox + y-flip + footprint box) but into the panel canvas.
function renderMap(p) {
    const x = p.ctx, cw = p.cw, ch = p.ch;
    p.bg();
    p.header('Map');
    const top = 56, area = ch - top - 16;
    const m = vui.lastMap;
    if (!m || !m.img) {
        x.fillStyle = C.dim; x.font = '16px ui-monospace,monospace';
        x.fillText('awaiting map…', 28, top + 30);
        return;
    }
    const scale = Math.min(cw / m.w, area / m.h);
    const dw = m.w * scale, dh = m.h * scale;
    const dx = (cw - dw) / 2, dy = top + (area - dh) / 2;
    x.imageSmoothingEnabled = false;
    x.save();
    x.translate(dx, dy + dh); x.scale(1, -1);  // world-north = top
    x.drawImage(m.img, 0, 0, dw, dh);
    const o = vui.lastOdom;
    if (o && m.res > 0) {
        const col = (o.x - m.origin[0]) / m.res, rw = (o.y - m.origin[1]) / m.res;
        const px = col * scale, py = rw * scale;
        if (px >= 0 && px <= dw && py >= 0 && py <= dh) {
            const pxPerM = scale / m.res;
            let lenPx = GO2_LEN_M * pxPerM, widPx = GO2_WID_M * pxPerM;
            if (lenPx < 14) { widPx *= 14 / lenPx; lenPx = 14; }
            x.save(); x.translate(px, py); x.rotate(o.yaw || 0);
            x.fillStyle = 'rgba(176,225,240,0.35)'; x.strokeStyle = C.cyan; x.lineWidth = 1.5;
            x.fillRect(-lenPx / 2, -widPx / 2, lenPx, widPx);
            x.strokeRect(-lenPx / 2, -widPx / 2, lenPx, widPx);
            x.beginPath(); x.moveTo(0, 0); x.lineTo(lenPx / 2, 0);
            x.strokeStyle = C.panel; x.lineWidth = 2; x.stroke();
            x.restore();
        }
    }
    const g = vui.navGoal;
    if (g && m.res > 0) {
        const gx = ((g.x - m.origin[0]) / m.res) * scale, gy = ((g.y - m.origin[1]) / m.res) * scale;
        x.save(); x.translate(gx, gy);
        x.strokeStyle = C.cyan; x.lineWidth = 1.5;
        x.beginPath(); x.arc(0, 0, 7, 0, Math.PI * 2); x.stroke();
        x.fillStyle = C.cyan; x.beginPath(); x.arc(0, 0, 2, 0, Math.PI * 2); x.fill();
        x.restore();
    }
    x.restore();
    // Whole map area is a click target → nav goal (store geometry for inverse).
    p._mapGeom = { dx, dy, dw, dh, scale, m };
    p.regions.push({ id: 'map', x: dx, y: top, w: dw, h: area });
}

// ── click dispatch ───────────────────────────────────────────────────
function handleButtonsClick(id, panel) {
    if (id === 'estop') return estop(panel);
    if (id === 'rearm') return rearm(panel);
    if (id === 'obstacle') return toggleObstacle(panel, id);
    const [kind, val] = id.split(':');
    if (kind === 'sport') sendSportCmd(val, panel, id);
    else if (kind === 'speed') selectSpeed(val);
    else if (kind === 'cam') toggleCam(val, panel);
    else if (kind === 'light') setLight(parseFloat(val), panel, id);
    panel.markDirty();
}

function handleMapClick(panel, uv) {
    const G = panel._mapGeom;
    if (!G || !chanReady()) return;
    // panel-uv → canvas px → map cell → world metres (inverse of renderMap).
    const px = uv.x * panel.cw, py = (1 - uv.y) * panel.ch;
    const col = (px - G.dx) / G.scale;
    const row = ((G.dy + G.dh) - py) / G.scale;
    if (col < 0 || col > G.dw / G.scale || row < 0 || row > G.dh / G.scale) return;
    const wx = G.m.origin[0] + col * G.m.res;
    const wy = G.m.origin[1] + row * G.m.res;
    vui.navGoal = { x: wx, y: wy };
    sendJSON({ type: 'nav_goal', x: wx, y: wy, nonce: ++vui.nonce });
    panel.markDirty();
}

// ─────────────────────────────────────────────────────────────────────
// Public: build the panels, wire click, expose per-frame redraw
// ─────────────────────────────────────────────────────────────────────
let _allPanels = [];
let _mapPanel = null;

export function buildCockpit(scene, headPos) {
    const buttons = new Panel({ wM: 0.7, hM: 1.15, cw: 512, ch: 900, opacity: 0.97 });
    const stats = new Panel({ wM: 0.5, hM: 1.0, cw: 380, ch: 820, opacity: 0.95 });
    const map = new Panel({ wM: 0.85, hM: 0.85, cw: 640, ch: 640, opacity: 0.97 });
    _mapPanel = map;

    // Arc facing the user: map left, buttons right, stats far-right column.
    map.place(new THREE.Vector3(-1.35, 1.5, -1.2), headPos);
    buttons.place(new THREE.Vector3(1.3, 1.45, -1.15), headPos);
    stats.place(new THREE.Vector3(2.0, 1.4, -0.35), headPos);
    for (const p of [map, buttons, stats]) { scene.add(p.mesh); p.mesh.renderOrder = 3; }

    _allPanels = [map, buttons, stats];
    buttons._render = renderButtons;
    stats._render = renderStats;
    map._render = renderMap;

    return {
        panels: _allPanels,
        meshes: _allPanels.map((p) => p.mesh),
        onClick(panel, uv) {
            if (panel === map) return handleMapClick(panel, uv);
            const id = panel.hitTest(uv);
            if (id) handleButtonsClick(id, panel);
        },
        // Redraw dirty panels + the always-live stats (health/telemetry tick).
        tick() {
            stats.markDirty();  // stats change continuously
            for (const p of _allPanels) {
                if (!p.dirty) continue;
                p._render(p);
                p.tex.needsUpdate = true;
                p.dirty = false;
            }
        },
        dispose() {
            for (const p of _allPanels) { scene.remove(p.mesh); p.dispose(); }
            _allPanels = []; _mapPanel = null;
        },
    };
}

export { SPEEDS };
