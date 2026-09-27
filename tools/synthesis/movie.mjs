// Records a video of Sphinx collecting fuel and shooting it, driven through the real robot code.
// A virtual Xbox controller is steered in a closed loop from the robot's position in the page.
//
//   node movie.mjs   (robot program on :3300, host site + relay on :4000, room SPHINX)
//
// Env: BASE, ROOM, STATION (red3), OUT (video folder), W/H (video size).
import { chromium } from "playwright"
import fs from "node:fs"

const E = process.env
const BASE = E.BASE ?? "http://localhost:4000", ROOM = E.ROOM ?? "SPHINX", STATION = E.STATION ?? "red3"
const OUT = E.OUT ?? "movie_out", W = +(E.W ?? 1600), H = +(E.H ?? 900)
fs.mkdirSync(OUT, { recursive: true })
const log = (...a) => console.log(new Date().toISOString().slice(11, 19), ...a)

const browser = await chromium.launch({ channel: "chrome", headless: true, args: ["--use-angle=metal", "--enable-webgl", "--ignore-gpu-blocklist"] })
const context = await browser.newContext({ viewport: { width: W, height: H }, recordVideo: { dir: OUT, size: { width: W, height: H } } })
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

// Facing +X is 0 rad; facing the red hub (-X) is pi.
const TOWARD_BLUE = 0, TOWARD_RED = Math.PI
await page.waitForTimeout(1500)
await go(-2.0, 3.45, TOWARD_BLUE, 1.0) // under the trench into the neutral zone
await buttons({ deploy: true, intake: true })
await go(-1.5, 1.6, TOWARD_BLUE, 0.8) // line up on the first row
await go(1.4, 1.6, TOWARD_BLUE, 0.7, 0.3) // sweep it
// Heading back toward red: move the camera behind the robot again (blue side).
await page.evaluate(() => window.__cam(0.4, -0.74, 5.2, 2000)) // high, from the +Z side: nothing between camera and robot
await go(1.9, 0.2, TOWARD_RED, 0.8, 0.35) // loop round
await go(-0.4, 0.2, TOWARD_RED, 0.7, 0.3) // sweep the second row back toward red
const loaded = await st()
log(`collected ${loaded.held} fuel`)
await page.waitForTimeout(600)
await buttons({ intake: false, deploy: false, stow: true })
// The hub only takes shots from inside your own alliance zone (a backboard blocks the
// neutral-zone side), so go home under the trench and shoot from about 3.3 m in front of it.
await go(-1.6, 3.45, TOWARD_RED, 0.9, 0.35)
await buttons({})
await go(-5.4, 3.45, TOWARD_RED, 1.0, 0.35)
await go(-6.0, 2.2, TOWARD_RED, 0.8, 0.3) // clear of the side wall before turning round
// High, from the +Z side inside the field: the robot below, the hub to its right.
await page.evaluate(() => window.__cam(0.17, -0.86, 4.6, 2500))
// 3.4 m from the hub and clear of the red tower: shooting from against the tower, a corner of
// the robot caught it while aiming and levered the robot up to 0.5 m sideways.
await go(-6.7, 1.6, TOWARD_BLUE, 0.6, 0.2)
await page.waitForTimeout(800)
const score = () => page.evaluate(() => document.body.innerText.match(/RED\s*(\d+)\s*BLUE\s*(\d+)/)?.slice(1).map(Number))
if (E.CAL) {
    // Calibration: 5 shots at each exit-speed gain, counting how many score.
    const results = []
    for (const gain of E.CAL.split(",").map(Number)) {
        await page.evaluate(g => { window.__X.shotTuning.gain = g }, gain)
        const s0 = await score(), h0 = (await st()).held
        if (h0 < 5) break
        const track = page.evaluate(() => window.__track(3500))
        await buttons({ shoot: true })
        for (let i = 0; i < 100 && (await st()).held > h0 - 5; i++) await page.waitForTimeout(50)
        await buttons({ shoot: false })
        await page.waitForTimeout(+(E.CALWAIT ?? 4000))
        const s1 = await score()
        const tr = await track
        // Balls that came down inside the opening (hub centre -3.65, 0; about 0.55 m across the flats).
        const into = tr.out.filter(c => Math.hypot(c[0] + 3.65, c[2]) < 0.55)
        const where = await page.evaluate(nodes => {
            const f = window.__W.sceneRenderer.mirabufSceneObjects.getField()
            return nodes.map(n => { const p = window.__W.physicsSystem.getBody(f.mechanism.nodeToBody.get(n)).GetCenterOfMassPosition(); return [+p.GetX().toFixed(2), +p.GetY().toFixed(2), +p.GetZ().toFixed(2)] })
        }, into.map(c => c[3]))
        results.push({ gain, fired: h0 - (await st()).held, scored: s1[0] - s0[0], wentIn: into.length, nowAt: where })
        log(JSON.stringify(results.at(-1)))
    }
    fs.writeFileSync(`${OUT}/cal.json`, JSON.stringify(results))
}
const before = await page.evaluate(() => document.body.innerText.match(/RED\s*(\d+)\s*BLUE\s*(\d+)/)?.slice(1))
const volley = page.evaluate(() => window.__track(14000))
await buttons({ shoot: true })
for (let i = 0; i < 25; i++) {
    await page.waitForTimeout(1000)
    const s = await st()
    if (s.held === 0) break
}
await page.waitForTimeout(3000)
await buttons({ shoot: false })
const after = await page.evaluate(() => document.body.innerText.match(/RED\s*(\d+)\s*BLUE\s*(\d+)/)?.slice(1))
const end = await st()
const v = await volley
const dropped = v.out.filter(c => Math.hypot(c[0] + 3.65, c[2]) < 0.45).length
log(`score before ${before} after ${after}; still holding ${end.held}; ${v.inHub} balls were inside the hub (${dropped} crossed the rim near the centre)`)
await page.waitForTimeout(2500)
const actionEnd = (Date.now() - t0) / 1000
const video = page.video()
await context.close()
await browser.close()
const path = await video.path()
fs.writeFileSync(`${OUT}/times.json`, JSON.stringify({ path, setupSecs, actionStart, actionEnd, collected: loaded.held, before, after }))
log("video", path, `action ${actionStart.toFixed(1)}-${actionEnd.toFixed(1)} s`)
