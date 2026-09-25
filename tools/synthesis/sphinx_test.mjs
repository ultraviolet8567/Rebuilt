// Headless end-to-end check of Sphinx in Synthesis driven by the 8567 robot code: spawn, attach,
// forward controllers from a fake gamepad, then exercise driving, the intake pivot and the hood.
// Prints one JSON line per step; screenshots to $SHOT_DIR.
import { chromium } from "playwright"

const ROOM = process.env.ROOM ?? "RBLT26"
const RELAY = process.env.RELAY ?? "10.211.55.5"
const CODESIM = process.env.CODESIM ?? "ws://localhost:3300/wpilibws"
const SPAWN = JSON.parse(process.env.SPAWN ?? "[6, 0.1, -2.5]")
const FIELD = process.env.FIELD !== "0"
const SHOT_DIR = process.env.SHOT_DIR ?? "."

const log = (tag, obj) => console.log(JSON.stringify({ t: new Date().toISOString().slice(11, 19), tag, ...obj }))
const browser = await chromium.launch({ channel: "chrome", headless: true, handleSIGTERM: false,
    args: ["--use-angle=metal", "--enable-webgl", "--ignore-gpu-blocklist"] })
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } })
page.on("pageerror", e => log("pageerror", { e: String(e).slice(0, 200) }))

await page.goto("http://localhost:3000/")
await page.waitForTimeout(3000)
await page.evaluate(async relay => {
    const P = (await import("/src/systems/preferences/PreferencesSystem.ts")).default
    P.setUserPreference("MultiplayerHost", relay)
    P.setUserPreference("MultiplayerPort", 9002)
    P.setUserPreference("MultiplayerSecure", false)
    P.setUserPreference("MultiplayerUsername", "Sphinx-test")
    P.savePreferences()
}, RELAY)
await page.goto(`http://localhost:3000/?autojoin=${ROOM}&codesim=${encodeURIComponent(CODESIM)}`)
await page.waitForTimeout(5000)

const setup = await page.evaluate(async ([field, spawn]) => {
    const X = await import("/src/dev/SwerveCodeSim.ts")
    const r = await X.setupPlayer(field, spawn, "sphinx")
    return { status: r.status, report: r.report }
}, [FIELD, SPAWN])
log("setup", setup)

// Fake gamepads + forwarding + teleop, and helpers kept on window for the steps below.
await page.evaluate(async () => {
    const mk = i => ({ id: `Test Xbox ${i}`, index: i, connected: true, mapping: "standard", timestamp: 0,
        axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => ({ pressed: false, touched: false, value: 0 })) })
    window.__pad = [mk(0), mk(1)]
    navigator.getGamepads = () => window.__pad
    const X = await import("/src/dev/SwerveCodeSim.ts")
    await X.startControllerForwarding()
    await X.setRobotMode("teleop")
    window.__X = X
})

const hinge = name => `(() => { const W = window.__W; const r = window.__X.myRobot();
    const d = W.simulationSystem.getSimulationLayer(r.mechanism).drivers.find(d => d.info?.name === "${name}");
    return +d.constraint.GetCurrentAngle().toFixed(3) })()`
await page.evaluate(async () => {
    const u = performance.getEntriesByType("resource").map(e => e.name).filter(n => /systems\/World\.ts/.test(n)).pop()
    window.__W = (await import(u)).default
})

async function step(name, pad, apply, ms) {
    return page.evaluate(async ([name, pad, apply, ms]) => {
        const X = window.__X, r = X.myRobot(), P = window.__pad[pad]
        const sleep = t => new Promise(res => setTimeout(res, t))
        const reset = () => { P.axes = [0, 0, 0, 0]; P.buttons.forEach(b => { b.pressed = false; b.value = 0 }) }
        const layer = window.__W.simulationSystem.getSimulationLayer(r.mechanism)
        const q = n => +layer.drivers.find(d => d.info?.name === n).constraint.GetCurrentAngle().toFixed(3)
        reset(); await sleep(600)
        const a = X.swerveSnapshot(r), qa = [q("dof_intake_pivot"), q("dof_hood")]
        for (const [k, v] of Object.entries(apply)) {
            if (k.startsWith("axis")) P.axes[+k.slice(4)] = v
            else { const b = P.buttons[+k.slice(3)]; b.pressed = true; b.value = 1 }
        }
        await sleep(ms)
        const mid = X.swerveSnapshot(r)
        const qm = [q("dof_intake_pivot"), q("dof_hood")]
        reset(); await sleep(600)
        const b = X.swerveSnapshot(r)
        const dx = b.pos[0] - a.pos[0], dz = b.pos[2] - a.pos[2]
        return { name, dirDeg: +(Math.atan2(dz, dx) * 180 / Math.PI).toFixed(0), dist: +Math.hypot(dx, dz).toFixed(2),
            speed: +Math.hypot(mid.vel[0], mid.vel[2]).toFixed(2), dyaw: +(b.yawDeg - a.yawDeg).toFixed(1),
            intake: [qa[0], qm[0]], hood: [qa[1], qm[1]] }
    }, [name, pad, apply, ms])
}

await page.waitForTimeout(2000)
log("idle", await step("idle", 0, {}, 3000))
log("drive", await step("fwd", 0, { axis1: -0.6 }, 1300))
log("drive", await step("left", 0, { axis0: -0.6 }, 1300))
log("drive", await step("rotCCW", 0, { axis2: -0.6 }, 1000))
log("drive", await step("fwd_after_turn", 0, { axis1: -0.6 }, 1300))
log("drive", await step("xlock", 0, { btn2: 1 }, 1000))
// Operator: Y stow, X middle, A deploy, D-pad up/down hood trim.
log("pivot", await step("stow_Y", 1, { btn3: 1 }, 2500))
await page.screenshot({ path: `${SHOT_DIR}/sphinx_stowed.png` })
log("pivot", await step("middle_X", 1, { btn2: 1 }, 2000))
log("pivot", await step("deploy_A", 1, { btn0: 1 }, 2500))
await page.screenshot({ path: `${SHOT_DIR}/sphinx_deployed.png` })
log("hood", await step("hood_up", 1, { btn12: 1 }, 2000))
log("hood", await step("hood_down", 1, { btn13: 1 }, 2000))

if (process.env.HOLD) {
    log("hold", { secs: +process.env.HOLD })
    await page.waitForTimeout(+process.env.HOLD * 1000)
}
await browser.close()
