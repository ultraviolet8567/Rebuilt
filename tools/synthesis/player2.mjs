// Player 2 for the multiplayer spike: a headless Chrome that joins room RBLT26 through the relay,
// spawns its own swerve robot, hands it to robot program #2 (ws://localhost:3301), then logs every
// second where it sees each robot. Stop it with SIGTERM.
import { chromium } from "playwright"

const ROOM = process.env.ROOM ?? "RBLT26"
const RELAY = process.env.RELAY ?? "10.211.55.5"
const CODESIM = process.env.CODESIM ?? "ws://localhost:3301/wpilibws"
const SPAWN = JSON.parse(process.env.SPAWN ?? "[6, 0.1, 0.5]")

const log = (...a) => console.log(new Date().toISOString().slice(11, 19), ...a)
const browser = await chromium.launch({ channel: "chrome", headless: true, handleSIGTERM: false, handleSIGINT: false, args: ["--use-angle=metal", "--enable-webgl", "--ignore-gpu-blocklist"] })
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } })
page.on("pageerror", e => log("pageerror", String(e).slice(0, 200)))

await page.goto("http://localhost:3000/")
await page.waitForTimeout(3000)
await page.evaluate(async ([relay]) => {
    const P = (await import("/src/systems/preferences/PreferencesSystem.ts")).default
    P.setUserPreference("MultiplayerHost", relay)
    P.setUserPreference("MultiplayerPort", 9002)
    P.setUserPreference("MultiplayerSecure", false)
    P.setUserPreference("MultiplayerUsername", "Player2-8567")
    P.savePreferences()
}, [RELAY])
await page.goto(`http://localhost:3000/?autojoin=${ROOM}&codesim=${encodeURIComponent(CODESIM)}`)
await page.waitForTimeout(5000)

const status = () => page.evaluate(async () => (await import("/src/dev/SwerveCodeSim.ts")).setupStatus())
log("joined", JSON.stringify(await status()))

// The field belongs to player 1; wait until it has arrived over the relay before spawning a robot.
for (let i = 0; i < 90 && !(await status()).field; i++) await page.waitForTimeout(1000)
log("field", JSON.stringify(await status()))
await page.evaluate(async () => (await import("/src/dev/SwerveCodeSim.ts")).startSetup(false))
for (let i = 0; i < 60; i++) {
    const s = await status()
    if (s.robots.some(r => r.startsWith("me:"))) break
    await page.waitForTimeout(1000)
}

// Close the tutorial and the post-spawn setup panel; the panel holds the physics paused.
for (const name of ["Skip", "Finish"]) {
    const b = page.getByRole("button", { name })
    if (await b.count()) await b.first().click().catch(() => {})
    await page.waitForTimeout(500)
}

const attach = await page.evaluate(async spawn => {
    const X = await import("/src/dev/SwerveCodeSim.ts")
    const W = (await import("/src/systems/World.ts")).default
    const r = X.myRobot()
    r.setObjectPosition({ pos: spawn, yaw: 0 })
    const rep = X.attachSwerveCodeSim(r)
    return { paused: W.physicsSystem.isPaused, modules: rep.map(o => `${o.module}:${o.hinge ?? "wheel"}`) }
}, SPAWN)
log("attached", JSON.stringify(attach), JSON.stringify(await status()))

// Enable teleop with sticks centred, re-asserted every loop, and report what this client sees.
const tick = () =>
    page.evaluate(async () => {
        const W = (await import("/src/systems/World.ts")).default
        const X = await import("/src/dev/SwerveCodeSim.ts")
        const { worker } = await import("/src/systems/simulation/wpilib_brain/WPILibTypes.ts")
        const send = (type, device, data) => worker.getValue().postMessage({ command: "update", data: { type, device, data } })
        send("DriverStation", "", { ">ds": true, ">enabled": true, ">autonomous": false, ">station": "red1", ">new_data": true })
        await X.setJoystick({})
        const robots = W.sceneRenderer.mirabufSceneObjects.getRobots().map(r => ({
            who: r.multiplayerOwnerName ?? "me",
            ...X.swerveSnapshot(r),
        }))
        return { paused: W.physicsSystem.isPaused, players: W.multiplayerSystem?.clientToInfoMap.size, robots }
    })

let stopping = false
let n = 0
process.on("SIGTERM", () => (stopping = true))
process.on("SIGINT", () => (stopping = true))
while (!stopping) {
    try {
        log("tick", JSON.stringify(await tick()))
        if (n++ % 10 === 5) await page.screenshot({ path: process.env.SHOT ?? "player2.png" })
    } catch (e) {
        log("tick error", String(e).slice(0, 200))
    }
    await page.waitForTimeout(1000)
}
await page.screenshot({ path: process.env.SHOT ?? "player2.png" })
await browser.close()
