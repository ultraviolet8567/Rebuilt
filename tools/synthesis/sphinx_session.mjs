// Long-lived headless Synthesis player for iterating: sets up Sphinx once, then watches
// ./probe/*.js; each new file is evaluated in the page and its result written to <file>.out.
import { chromium } from "playwright"
import fs from "node:fs"
const RELAY = process.env.RELAY ?? "10.211.55.5", ROOM = process.env.ROOM ?? "RBLT26"
const CODESIM = process.env.CODESIM ?? "ws://localhost:3300/wpilibws"
const PROBE = process.env.PROBE_DIR ?? "probe", NAME = process.env.NAME ?? "Sphinx-test"
const FIELD = process.env.FIELD !== "0", SPAWN = JSON.parse(process.env.SPAWN ?? "[6, 0.1, -2.5]")
const browser = await chromium.launch({ channel: "chrome", headless: true, handleSIGTERM: false,
    args: ["--use-angle=metal", "--enable-webgl", "--ignore-gpu-blocklist", "--enable-gpu"] })
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } })
await page.goto("http://localhost:3000/"); await page.waitForTimeout(3000)
await page.evaluate(async ([relay, name]) => {
    const P = (await import("/src/systems/preferences/PreferencesSystem.ts")).default
    P.setUserPreference("MultiplayerHost", relay); P.setUserPreference("MultiplayerPort", 9002)
    P.setUserPreference("MultiplayerSecure", false); P.setUserPreference("MultiplayerUsername", name); P.savePreferences()
}, [RELAY, NAME])
await page.goto(`http://localhost:3000/?autojoin=${ROOM}&codesim=${encodeURIComponent(CODESIM)}`)
await page.waitForTimeout(5000)
const setup = await page.evaluate(async ([field, spawn]) => {
    const X = await import("/src/dev/SwerveCodeSim.ts")
    const r = await X.setupPlayer(field, spawn, "sphinx")
    const mk = i => ({ id: `Test Xbox ${i}`, index: i, connected: true, mapping: "standard", timestamp: 0,
        axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => ({ pressed: false, touched: false, value: 0 })) })
    window.__pad = [mk(0), mk(1)]; navigator.getGamepads = () => window.__pad
    await X.startControllerForwarding(); await X.setRobotMode("teleop"); window.__X = X
    const u = performance.getEntriesByType("resource").map(e => e.name).filter(n => /systems\/World\.ts/.test(n)).pop()
    window.__W = (await import(u)).default
    return r.status
}, [FIELD, SPAWN])
fs.mkdirSync(PROBE, { recursive: true })
fs.writeFileSync(PROBE + "/READY", JSON.stringify(setup))
const done = new Set()
for (;;) {
    for (const f of fs.readdirSync(PROBE).filter(f => f.endsWith(".js"))) {
        if (done.has(f)) continue
        done.add(f)
        let out
        try { out = await page.evaluate(`(async () => { ${fs.readFileSync(PROBE + "/" + f, "utf8")} })()`) }
        catch (e) { out = { error: String(e).slice(0, 500) } }
        if (f.includes("shot")) await page.screenshot({ path: `${PROBE}/${f}.png` })
        fs.writeFileSync(`${PROBE}/${f}.out`, JSON.stringify(out))
    }
    if (fs.existsSync(PROBE + "/STOP")) break
    await new Promise(r => setTimeout(r, 300))
}
await browser.close()
