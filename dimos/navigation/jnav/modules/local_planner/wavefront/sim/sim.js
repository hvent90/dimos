// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
//
// Repulsive-field local planner — web sim frontend.
// Paints a 2D costmap, sends it (+ goal + facing params) to the Python backend
// over a websocket, and renders the oriented local path the real planner returns.

"use strict"

/** @typedef {{cols:number, rows:number, resolution:number, origin:[number,number]}} MapMeta */

const CELL_PX = 11 // canvas pixels per grid cell

const state = {
    cols: 70,
    rows: 50,
    resolution: 0.1, // metres per cell
    origin: [0, 0], // world (x,y) of cell (row 0, col 0)
    /** @type {Uint8Array} 0 = free, 100 = lethal, row-major (row*cols + col) */
    grid: new Uint8Array(70 * 50),
    start: [0.6, 2.5], // world metres
    goal: [6.0, 2.5], // world metres
    poses: [], // [[x,y,yaw], ...] from backend
    globalPath: [], // [[x,y], ...] from backend
    faceForward: 0.8,
    omnidirectional: false,
    influence: 0.8,
    vehicleWidth: 0.5,
    safetyMargin: 0.0,
    commitmentWeight: 0.0,
    brush: 2,
    painting: false,
    erasing: false,
    // dynamic (moving) obstacles: each patrols a<->b on a period (seconds)
    /** @type {{ax:number,ay:number,bx:number,by:number,hx:number,hy:number,period:number}[]} */
    dynamicObstacles: [],
    dynamicOn: false,
    prevPath: null, // [[x,y],...] last local path, fed back for temporal commitment
}

// current world-frame boxes for the moving obstacles (triangle-wave a<->b)
function obstacleBoxesNow() {
    const tSec = (typeof performance !== "undefined" ? performance.now() : Date.now()) / 1000
    return state.dynamicObstacles.map((o) => {
        const phase = (tSec % o.period) / o.period // 0..1
        const tri = phase < 0.5 ? phase * 2 : 2 - phase * 2 // 0..1..0
        return { x: o.ax + (o.bx - o.ax) * tri, y: o.ay + (o.by - o.ay) * tri, hx: o.hx, hy: o.hy }
    })
}

// static painted grid + the moving obstacles painted on top
function compositeGrid() {
    const g = state.grid.slice()
    if (!state.dynamicOn) { return g }
    for (const b of obstacleBoxesNow()) {
        const c0 = Math.round((b.x - b.hx - state.origin[0]) / state.resolution)
        const c1 = Math.round((b.x + b.hx - state.origin[0]) / state.resolution)
        const r0 = Math.round((b.y - b.hy - state.origin[1]) / state.resolution)
        const r1 = Math.round((b.y + b.hy - state.origin[1]) / state.resolution)
        for (let r = Math.max(r0, 0); r < Math.min(r1, state.rows); r++) {
            for (let c = Math.max(c0, 0); c < Math.min(c1, state.cols); c++) {
                g[r * state.cols + c] = 100
            }
        }
    }
    return g
}

const canvas = /** @type {HTMLCanvasElement} */ (document.getElementById("map"))
const ctx = canvas.getContext("2d")
const statusEl = document.getElementById("status")

// ---- coordinate transforms -------------------------------------------------
// World y is up; canvas y is down, so we flip when drawing.
function worldToCanvas(x, y) {
    const col = (x - state.origin[0]) / state.resolution
    const row = (y - state.origin[1]) / state.resolution
    return [col * CELL_PX, (state.rows - row) * CELL_PX]
}
function canvasToCell(px, py) {
    const col = Math.floor(px / CELL_PX)
    const row = Math.floor(state.rows - py / CELL_PX)
    return [col, row]
}
function cellToWorld(col, row) {
    return [state.origin[0] + (col + 0.5) * state.resolution,
            state.origin[1] + (row + 0.5) * state.resolution]
}

// ---- websocket -------------------------------------------------------------
let ws = null
let wantPlan = false
let inFlight = false

function connect() {
    const url = `ws://${location.hostname}:8765`
    ws = new WebSocket(url)
    ws.onopen = () => { setStatus("connected — paint obstacles, right-click sets goal"); requestPlan() }
    ws.onclose = () => { setStatus("disconnected — is server.py running?"); setTimeout(connect, 1500) }
    ws.onerror = () => setStatus("websocket error")
    ws.onmessage = (ev) => {
        inFlight = false
        const msg = JSON.parse(ev.data)
        if (msg.type === "path") {
            state.poses = msg.poses
            state.globalPath = msg.global_path
            state.prevPath = msg.poses.length > 1 ? msg.poses.map((p) => [p[0], p[1]]) : null
            render()
        } else if (msg.type === "error") {
            setStatus("backend error: " + msg.message)
        }
        if (wantPlan) { wantPlan = false; sendPlan() }
    }
}

function requestPlan() {
    // Coalesce rapid edits into one in-flight request at a time.
    if (inFlight) { wantPlan = true; return }
    sendPlan()
}
function sendPlan() {
    if (!ws || ws.readyState !== WebSocket.OPEN) { return }
    inFlight = true
    ws.send(JSON.stringify({
        width: state.cols,
        height: state.rows,
        resolution: state.resolution,
        origin: state.origin,
        grid: Array.from(compositeGrid()),
        start: [state.start[0], state.start[1], 0.0],
        goal: state.goal,
        previous_path: state.dynamicOn ? state.prevPath : null,
        params: {
            face_forward_weight: state.faceForward,
            omnidirectional: state.omnidirectional,
            influence_radius: state.influence,
            vehicle_width: state.vehicleWidth,
            safety_margin: state.safetyMargin,
            commitment_weight: state.commitmentWeight,
        },
    }))
}

function setStatus(text) { statusEl.textContent = text }

// ---- rendering -------------------------------------------------------------
function render() {
    canvas.width = state.cols * CELL_PX
    canvas.height = state.rows * CELL_PX
    ctx.clearRect(0, 0, canvas.width, canvas.height)

    // grid background + obstacles
    for (let row = 0; row < state.rows; row++) {
        for (let col = 0; col < state.cols; col++) {
            const v = state.grid[row * state.cols + col]
            ctx.fillStyle = v >= 50 ? "#cf4f5a" : ((row + col) % 2 ? "#10141b" : "#0d1118")
            const [cx, cy] = worldToCanvas(...cellToWorld(col, row))
            ctx.fillRect(cx - CELL_PX / 2, cy - CELL_PX / 2, CELL_PX, CELL_PX)
        }
    }

    // moving obstacles (orange) on top of the static grid
    if (state.dynamicOn) {
        ctx.fillStyle = "#ff8c2b"
        for (const b of obstacleBoxesNow()) {
            const [cx, cy] = worldToCanvas(b.x, b.y)
            const w = (b.hx * 2 / state.resolution) * CELL_PX
            const h = (b.hy * 2 / state.resolution) * CELL_PX
            ctx.fillRect(cx - w / 2, cy - h / 2, w, h)
        }
    }

    // global path (faint dashed)
    if (state.globalPath.length > 1) {
        ctx.strokeStyle = "#445"
        ctx.lineWidth = 1.5
        ctx.setLineDash([5, 5])
        ctx.beginPath()
        state.globalPath.forEach(([x, y], i) => {
            const [cx, cy] = worldToCanvas(x, y)
            i ? ctx.lineTo(cx, cy) : ctx.moveTo(cx, cy)
        })
        ctx.stroke()
        ctx.setLineDash([])
    }

    // vehicle footprint swept along the local path (translucent band of body width)
    if (state.poses.length > 1 && state.vehicleWidth > 0) {
        ctx.strokeStyle = "rgba(78,161,255,0.18)"
        ctx.lineWidth = (state.vehicleWidth / state.resolution) * CELL_PX
        ctx.lineCap = "round"
        ctx.lineJoin = "round"
        ctx.beginPath()
        state.poses.forEach(([x, y], i) => {
            const [cx, cy] = worldToCanvas(x, y)
            i ? ctx.lineTo(cx, cy) : ctx.moveTo(cx, cy)
        })
        ctx.stroke()
        ctx.lineCap = "butt"
    }

    // local path (solid blue) + heading arrows
    if (state.poses.length > 1) {
        ctx.strokeStyle = "#4ea1ff"
        ctx.lineWidth = 2.5
        ctx.beginPath()
        state.poses.forEach(([x, y], i) => {
            const [cx, cy] = worldToCanvas(x, y)
            i ? ctx.lineTo(cx, cy) : ctx.moveTo(cx, cy)
        })
        ctx.stroke()
        const step = Math.max(1, Math.round(state.poses.length / 18))
        for (let i = 0; i < state.poses.length; i += step) { drawArrow(state.poses[i]) }
    }

    drawStart()
    drawGoal()
}

function drawArrow([x, y, yaw]) {
    const [cx, cy] = worldToCanvas(x, y)
    const len = CELL_PX * 1.6
    const ex = cx + len * Math.cos(yaw)
    const ey = cy - len * Math.sin(yaw) // canvas y is flipped
    ctx.strokeStyle = "#ffd24e"
    ctx.fillStyle = "#ffd24e"
    ctx.lineWidth = 1.6
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke()
    const a = Math.atan2(ey - cy, ex - cx)
    ctx.beginPath()
    ctx.moveTo(ex, ey)
    ctx.lineTo(ex - 5 * Math.cos(a - 0.5), ey - 5 * Math.sin(a - 0.5))
    ctx.lineTo(ex - 5 * Math.cos(a + 0.5), ey - 5 * Math.sin(a + 0.5))
    ctx.closePath(); ctx.fill()
}

function drawStart() {
    const [cx, cy] = worldToCanvas(...state.start)
    ctx.fillStyle = "#37d67a"
    ctx.beginPath(); ctx.arc(cx, cy, 6, 0, Math.PI * 2); ctx.fill()
}
function drawGoal() {
    const [cx, cy] = worldToCanvas(...state.goal)
    ctx.fillStyle = "#ff5b5b"
    ctx.beginPath(); ctx.arc(cx, cy, 6, 0, Math.PI * 2); ctx.fill()
    ctx.strokeStyle = "#ff5b5b"; ctx.lineWidth = 2
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - 16); ctx.stroke()
}

// ---- painting --------------------------------------------------------------
function paintAt(px, py) {
    const [c0, r0] = canvasToCell(px, py)
    const b = state.brush - 1
    let changed = false
    for (let dr = -b; dr <= b; dr++) {
        for (let dc = -b; dc <= b; dc++) {
            const col = c0 + dc, row = r0 + dr
            if (col < 0 || row < 0 || col >= state.cols || row >= state.rows) { continue }
            const idx = row * state.cols + col
            const v = state.erasing ? 0 : 100
            if (state.grid[idx] !== v) { state.grid[idx] = v; changed = true }
        }
    }
    if (changed) { render(); requestPlan() }
}

canvas.addEventListener("contextmenu", (e) => e.preventDefault())
canvas.addEventListener("mousedown", (e) => {
    const r = canvas.getBoundingClientRect()
    const px = e.clientX - r.left, py = e.clientY - r.top
    if (e.button === 2) { // right-click: goal
        state.goal = canvasWorldClamped(px, py)
        render(); requestPlan()
    } else if (e.button === 0) { // left: paint
        state.painting = true
        state.erasing = e.shiftKey
        paintAt(px, py)
    }
})
canvas.addEventListener("mousemove", (e) => {
    if (!state.painting) { return }
    const r = canvas.getBoundingClientRect()
    paintAt(e.clientX - r.left, e.clientY - r.top)
})
window.addEventListener("mouseup", () => { state.painting = false })

function canvasWorldClamped(px, py) {
    let [col, row] = canvasToCell(px, py)
    col = Math.min(Math.max(col, 0), state.cols - 1)
    row = Math.min(Math.max(row, 0), state.rows - 1)
    return cellToWorld(col, row)
}

// ---- controls --------------------------------------------------------------
function bind(id, fn) { document.getElementById(id).addEventListener("input", fn) }

bind("faceForward", (e) => {
    state.faceForward = parseFloat(e.target.value)
    document.getElementById("ffVal").textContent = state.faceForward.toFixed(2)
    requestPlan()
})
bind("omni", (e) => { state.omnidirectional = e.target.checked; requestPlan() })
bind("influence", (e) => {
    state.influence = parseFloat(e.target.value)
    document.getElementById("infVal").textContent = state.influence.toFixed(2)
    requestPlan()
})
bind("vehicleWidth", (e) => {
    state.vehicleWidth = parseFloat(e.target.value)
    document.getElementById("vwVal").textContent = state.vehicleWidth.toFixed(2)
    requestPlan()
})
bind("brush", (e) => {
    state.brush = parseInt(e.target.value, 10)
    document.getElementById("brushVal").textContent = String(state.brush)
})
document.getElementById("clearBtn").addEventListener("click", () => {
    state.grid.fill(0); render(); requestPlan()
})
window.addEventListener("keydown", (e) => {
    if (e.key === "c" || e.key === "C") { state.grid.fill(0); render(); requestPlan() }
})

bind("safetyMargin", (e) => {
    state.safetyMargin = parseFloat(e.target.value)
    document.getElementById("smVal").textContent = state.safetyMargin.toFixed(2)
    requestPlan()
})
bind("commitment", (e) => {
    state.commitmentWeight = parseFloat(e.target.value)
    document.getElementById("cmVal").textContent = state.commitmentWeight.toFixed(1)
    requestPlan()
})

// two default patrol boxes crossing the path in opposite phase
function seedDynamicObstacles() {
    const midx = (state.start[0] + state.goal[0]) / 2
    const lo = state.origin[1] + state.rows * state.resolution * 0.12
    const hi = state.origin[1] + state.rows * state.resolution * 0.88
    state.dynamicObstacles = [
        { ax: midx - 1.2, ay: lo, bx: midx - 1.2, by: hi, hx: 0.3, hy: 0.3, period: 7 },
        { ax: midx + 1.2, ay: hi, bx: midx + 1.2, by: lo, hx: 0.3, hy: 0.3, period: 9 },
    ]
}
const dynCheckbox = document.getElementById("dynamic")
if (dynCheckbox) {
    dynCheckbox.addEventListener("input", (e) => {
        state.dynamicOn = e.target.checked
        if (state.dynamicOn && state.dynamicObstacles.length === 0) { seedDynamicObstacles() }
        state.prevPath = null
        render()
    })
}

// animation: move the obstacles and replan while dynamic mode is on
setInterval(() => {
    if (!state.dynamicOn) { return }
    requestPlan()
    render()
}, 90)

// ---- URL-defined scenario --------------------------------------------------
// ?map=<url-encoded JSON>:
//   { cols, rows, resolution, origin:[x,y], start:[x,y], goal:[x,y],
//     rects:[[col,row,wCells,hCells], ...] }   // filled lethal rectangles
function loadFromUrl() {
    const raw = new URLSearchParams(location.search).get("map")
    if (!raw) { return }
    let spec
    try { spec = JSON.parse(raw) } catch (err) { setStatus("bad ?map JSON: " + err); return }
    if (spec.cols) { state.cols = spec.cols }
    if (spec.rows) { state.rows = spec.rows }
    if (spec.resolution) { state.resolution = spec.resolution }
    if (spec.origin) { state.origin = spec.origin }
    state.grid = new Uint8Array(state.cols * state.rows)
    if (spec.start) { state.start = spec.start }
    if (spec.goal) { state.goal = spec.goal }
    for (const [col, row, w, h] of spec.rects || []) {
        for (let r = row; r < row + h; r++) {
            for (let c = col; c < col + w; c++) {
                if (c >= 0 && r >= 0 && c < state.cols && r < state.rows) {
                    state.grid[r * state.cols + c] = 100
                }
            }
        }
    }
    // dynamic obstacles + params: { dyn:[{ax,ay,bx,by,hx,hy,period}], safety_margin, commitment_weight }
    if (spec.dyn) {
        state.dynamicObstacles = spec.dyn
        state.dynamicOn = true
        const cb = document.getElementById("dynamic"); if (cb) { cb.checked = true }
    }
    if (spec.safety_margin != null) { state.safetyMargin = spec.safety_margin }
    if (spec.commitment_weight != null) { state.commitmentWeight = spec.commitment_weight }
}

loadFromUrl()
render()
connect()
