// Calibrates the simulator's shot table (sim.json launcher.efficiencyByRpm) against the robot
// code's own distance -> rpm table. At each distance the robot code aims and picks the rpm; a
// speed gain is searched until the balls come down through rim height at the hub centre. The new
// table value at that rpm is the old one times the gain.
//
//   node calib.mjs   (robot program on :3300, host site + relay on :4000, room SPHINX)
// Env: BASE, ROOM, STATION (red3), DISTS (metres, comma separated), SHOTS per try (3).
import { chromium } from "playwright"
import fs from "node:fs"

const E = process.env
const BASE = E.BASE ?? "http://localhost:4000", ROOM = E.ROOM ?? "SPHINX", STATION = E.STATION ?? "red3"
const OUT = E.OUT ?? "movie_out", W = +(E.W ?? 1600), H = +(E.H ?? 900)
fs.mkdirSync(OUT, { recursive: true })
const log = (...a) => console.log(new Date().toISOString().slice(11, 19), ...a)

const browser = await chromium.launch({ channel: "chrome", headless: true, args: ["--use-angle=metal", "--enable-webgl", "--ignore-gpu-blocklist"] })
const context = await browser.newContext({ viewport: { width: W, height: H } })
const page = await context.newPage()
await page.addInitScript(() => {
    const pad = { id: "Virtual Xbox", index: 0, connected: true, mapping: "standard", timestamp: 0,
        axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => ({ pressed: false, touched: false, value: 0 })) }
    window.__pad = [pad]
    navigator.getGamepads = () => window.__pad
})
const relay = BASE.replace(/^http/, "ws")
const t0 = Date.now()
await page.goto(`${BASE}/?autojoin=${ROOM}&sphinx=${STATION}&relay=${encodeURIComponent(relay)}&field=1&view=follow`)
await page.waitForFunction(() => window.__sphinx, null, { timeout: 240000, polling: 1000 })
const status = await page.evaluate(() => window.__sphinx)
if (!status.ready) throw new Error("setup failed: " + status.error)
const setupSecs = (Date.now() - t0) / 1000
// Close the analytics cookie box with its decline (x) button so it is not in the video.
await page.evaluate(() => {
    const box = [...document.querySelectorAll("div")].reverse().find(d => d.innerText?.startsWith("Synthesis uses cookies"))
    const decline = box && [...box.querySelectorAll("button")].find(b => !/consent/i.test(b.innerText))
    decline?.click()
})
log(`ready after ${setupSecs.toFixed(0)} s`)
log("field owner check", JSON.stringify(await page.evaluate(() => {
    const W = window.__W, f = W.sceneRenderer.mirabufSceneObjects.getField()
    return { me: W.multiplayerSystem?.clientId, fieldOwner: f?._multiplayerOwningClientId, host: W.multiplayerSystem?.isHost }
})))

// In-page controller: drives to waypoints with field-relative sticks and holds a heading.
await page.evaluate(() => {
    const W = window.__W, X = window.__X, pad = window.__pad[0]
    const robot = X.myRobot()
    const red = robot.alliance === "red"
    const body = () => W.physicsSystem.getBody(robot.mechanism.nodeToBody.get(robot.rootNodeId))
    window.__state = () => {
        const b = body(), p = b.GetPosition(), q = b.GetRotation()
        // Chassis forward is local +X; rotate it by the body quaternion.
        const [x, y, z, w] = [q.GetX(), q.GetY(), q.GetZ(), q.GetW()]
        const fx = 1 - 2 * (y * y + z * z), fz = 2 * (x * z - w * y)
        return { x: p.GetX(), z: p.GetZ(), heading: Math.atan2(-fz, fx), held: robot._ejectables?.length ?? 0 }
    }
    const btn = (i, on) => { pad.buttons[i] = { pressed: on, touched: on, value: on ? 1 : 0 } }
    window.__buttons = ({ intake = false, deploy = false, shoot = false, stow = false } = {}) => {
        btn(0, deploy || intake) // A: deploy intake
        btn(4, intake) // LB: rollers in
        btn(3, stow) // Y: stow
        btn(7, shoot) // RT: aim at hub and shoot
    }
    // Orbit camera around the robot: theta/phi as Synthesis defines them, r in metres.
    window.__cam = (theta, phi, r, ms = 1500) => {
        const c = W.sceneRenderer.currentCameraControls
        c.setImmediateCoordinates({ r })
        c.animateToOrientation(theta, phi, ms)
    }
    // Where airborne fuel comes down through the hub's rim height (1.83 m) over the next ms.
    window.__track = ms => new Promise(resolve => {
        const f = W.sceneRenderer.mirabufSceneObjects.getField(), last = new Map(), out = [], inHub = new Set()
        const t0 = performance.now()
        const id = setInterval(() => {
            for (const [node, bid] of f.mechanism.nodeToBody) {
                if (!String(node).endsWith("_gp")) continue
                const p = W.physicsSystem.getBody(bid).GetCenterOfMassPosition()
                const cur = [p.GetX(), p.GetY(), p.GetZ()], prev = last.get(node)
                if (prev && prev[1] >= 1.83 && cur[1] < 1.83) out.push([...cur.map(v => +v.toFixed(2)), node])
                // Inside a hub: within 0.42 m of its centre axis and below the rim.
                for (const hx of [-3.65, 3.65]) if (Math.hypot(cur[0] - hx, cur[2]) < 0.42 && cur[1] < 1.75 && cur[1] > 0.4) inHub.add(node)
                // Full path of the first ball to leave the robot.
                if (!window.__path && prev && cur[1] > 0.9 && cur[1] > prev[1] + 0.02) { window.__path = []; window.__pathNode = node }
                if (window.__pathNode === node) window.__path.push([+(performance.now() - t0).toFixed(0), ...cur.map(v => +v.toFixed(2))])
                last.set(node, cur)
            }
            if (performance.now() - t0 > ms) { clearInterval(id); const path = window.__path; window.__path = undefined; window.__pathNode = undefined; resolve({ out, inHub: inHub.size, path: path?.filter((_, i) => i % 3 === 0) }) }
        }, 16)
    })
    const wrap = a => Math.atan2(Math.sin(a), Math.cos(a))
    window.__goto = (tx, tz, heading, speed = 0.55, tol = 0.25) =>
        new Promise(resolve => {
            const t0 = performance.now()
            const id = setInterval(() => {
                const s = window.__state()
                const dx = tx - s.x, dz = tz - s.z, d = Math.hypot(dx, dz)
                const k = Math.min(speed, 0.35 + d * 0.6) / Math.max(d, 1e-3)
                // Field-relative sticks, from this alliance's driver station: up = away from own wall.
                const fwd = (red ? dx : -dx) * k, right = (red ? dz : -dz) * k
                const eh = heading === undefined ? 0 : wrap(heading - s.heading)
                pad.axes[0] = Math.max(-1, Math.min(1, right))
                pad.axes[1] = Math.max(-1, Math.min(1, -fwd))
                pad.axes[2] = Math.max(-0.6, Math.min(0.6, -eh * 1.5))
                const done = (d < tol && Math.abs(eh) < 0.3) || performance.now() - t0 > 20000
                if (done) {
                    clearInterval(id)
                    pad.axes[0] = pad.axes[1] = pad.axes[2] = 0
                    resolve({ ...window.__state(), ms: Math.round(performance.now() - t0) })
                }
            }, 20)
        })
})

const st = () => page.evaluate(() => window.__state())
const go = async (x, z, heading, speed, tol) => {
    const r = await page.evaluate(([x, z, h, s, t]) => window.__goto(x, z, h ?? undefined, s, t), [x, z, heading ?? null, speed ?? 0.55, tol ?? 0.25])
    log(`at (${r.x.toFixed(2)}, ${r.z.toFixed(2)}) heading ${(r.heading * 57.3).toFixed(0)} deg, holding ${r.held}, ${r.ms} ms`)
    return r
}
const buttons = b => page.evaluate(b => window.__buttons(b), b)
const s0 = await st()
log("start", JSON.stringify(s0))
const actionStart = (Date.now() - t0) / 1000

// Robot code (ShooterConstants.kDistanceToRpm) and the current sim table, for the report.
const CODE_RPM = [[1.5, 2927], [2, 3338], [2.5, 3685], [3, 3968], [3.5, 4187], [4, 4342], [4.5, 4433], [5, 4590], [5.5, 4732], [6, 4861], [7, 5091], [8, 5290]]
const lerp = (t, x) => {
    if (x <= t[0][0]) return t[0][1]
    for (let i = 1; i < t.length; i++) if (x <= t[i][0]) { const [a, b] = [t[i - 1], t[i]]; return a[1] + ((b[1] - a[1]) * (x - a[0])) / (b[0] - a[0]) }
    return t.at(-1)[1]
}
const OLD = JSON.parse(E.OLD_TABLE ?? "[[2927,0.49],[3604,0.40],[3833,0.352],[4130,0.337],[4234,0.329],[5290,0.263]]")
const HUB = [-3.65, 0]
const SHOTS = +(E.SHOTS ?? 3)

// Rim-height (1.83 m) crossings on the way down near the hub, in the next `ms`.
await page.evaluate(() => {
    const W = window.__W
    window.__crossings = ms => new Promise(resolve => {
        const f = W.sceneRenderer.mirabufSceneObjects.getField(), last = new Map(), out = []
        const t0 = performance.now()
        const id = setInterval(() => {
            for (const [node, bid] of f.mechanism.nodeToBody) {
                if (!String(node).endsWith("_gp")) continue
                const p = W.physicsSystem.getBody(bid).GetCenterOfMassPosition()
                const cur = [p.GetX(), p.GetY(), p.GetZ()], prev = last.get(node)
                if (prev && prev[1] >= 1.83 && cur[1] < 1.83 && Math.hypot(cur[0] + 3.65, cur[2]) < 3) out.push([cur[0], cur[2]])
                last.set(node, cur)
            }
            if (performance.now() - t0 > ms) { clearInterval(id); resolve(out) }
        }, 10)
    })
})

const median = a => { const s = [...a].sort((x, y) => x - y); return s.length ? s[Math.floor(s.length / 2)] : undefined }

async function volley(gain) {
    await page.evaluate(g => { window.__X.shotTuning.gain = g }, gain)
    await buttons({ deploy: true, intake: true })
    const held = await page.evaluate(n => window.__X.feedIntakeForTest(n), SHOTS)
    await page.waitForTimeout(500)
    await buttons({ stow: true })
    await page.waitForTimeout(600)
    await buttons({})
    const s = await st()
    const h0 = s.held
    const cross = page.evaluate(() => window.__crossings(6000))
    await buttons({ shoot: true })
    for (let i = 0; i < 200 && (await st()).held > h0 - SHOTS; i++) await page.waitForTimeout(50)
    await page.waitForTimeout(300)
    await buttons({})
    const c = await cross
    // Signed error along the line of fire: + long, - short. No rim crossing = well short.
    const r = await st()
    const u = [HUB[0] - r.x, HUB[1] - r.z], n = Math.hypot(...u)
    const along = c.map(p => ((p[0] - HUB[0]) * u[0] + (p[1] - HUB[1]) * u[1]) / n)
    const err = along.length ? median(along) : -1.5
    return { gain, held, fired: h0 - (await st()).held, crossings: along.map(v => +v.toFixed(2)), err: +err.toFixed(3), dist: +n.toFixed(2) }
}

const results = []
for (const d of (E.DISTS ?? "2.0,2.5,3.0,3.5,4.0,4.4").split(",").map(Number)) {
    // On a 30 degree line from the hub, which keeps the robot clear of the tower.
    const a = Math.PI / 6, x = HUB[0] - d * Math.cos(a), z = d * Math.sin(a)
    await go(x, z, Math.atan2(-(HUB[1] - z), HUB[0] - x), 0.6, 0.08)
    await page.waitForTimeout(800)
    const tries = []
    let g0 = 1, t0 = await volley(g0)
    tries.push(t0)
    let g1 = t0.err > 0 ? 0.95 : 1.05, t1 = await volley(g1)
    tries.push(t1)
    for (let k = 0; k < 5 && Math.abs(t1.err) > 0.08; k++) {
        const slope = (t1.err - t0.err) / (g1 - g0)
        let g2 = Math.abs(slope) > 1e-3 ? g1 - t1.err / slope : g1 * (t1.err > 0 ? 0.95 : 1.05)
        g2 = Math.min(1.8, Math.max(0.5, g2))
        ;[g0, t0] = [g1, t1]
        g1 = g2
        t1 = await volley(g1)
        tries.push(t1)
    }
    const best = tries.reduce((b, t) => (Math.abs(t.err) < Math.abs(b.err) ? t : b))
    const rpm = lerp(CODE_RPM, best.dist)
    const row = { dist: best.dist, rpm: Math.round(rpm), gain: best.gain, err: best.err, oldEff: +lerp(OLD, rpm).toFixed(4), newEff: +(lerp(OLD, rpm) * best.gain).toFixed(4), tries }
    results.push(row)
    log(JSON.stringify({ ...row, tries: tries.map(t => [+t.gain.toFixed(3), t.err]) }))
}
fs.writeFileSync(`${OUT}/calib.json`, JSON.stringify(results, null, 1))
await browser.close()
