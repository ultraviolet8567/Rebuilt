// Scripted player using the same one-link start a person would: /?autojoin=ROOM&sphinx=STATION.
// Adds fake gamepads, then serves probe snippets from $PROBE_DIR like sphinx_session.mjs.
import { chromium } from "playwright"
import fs from "node:fs"
const E = process.env
const RELAY = E.RELAY ?? "10.211.55.5", ROOM = E.ROOM ?? "RBLT26", STATION = E.STATION ?? "blue1"
const CODESIM = E.CODESIM ?? "ws://localhost:3300/wpilibws", PROBE = E.PROBE_DIR ?? "probe"
const BASE = E.BASE ?? "http://localhost:3000"
const browser = await chromium.launch({ channel: "chrome", headless: true, handleSIGTERM: false,
    // A scripted player cannot click Chrome's "access devices on your local network" prompt, which
    // an https page (the internet tunnel) needs to reach the robot program on localhost.
    args: ["--use-angle=metal", "--enable-webgl", "--ignore-gpu-blocklist", "--disable-features=LocalNetworkAccessChecks"] })
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } })
await page.goto(BASE + "/"); await page.waitForTimeout(3000)
await page.evaluate(([relay, name, port, secure]) => {
    const prefs = JSON.parse(localStorage.getItem("Preferences") ?? "{}")
    prefs.MultiplayerHost = relay; prefs.MultiplayerPort = port; prefs.MultiplayerSecure = secure; prefs.MultiplayerUsername = name
    localStorage.setItem("Preferences", JSON.stringify(prefs))
}, [RELAY, E.NAME ?? `8567-${STATION}`, +(E.RELAY_PORT ?? 9002), E.SECURE === "1"])
await page.addInitScript(() => {
    const mk = i => ({ id: `Test Xbox ${i}`, index: i, connected: true, mapping: "standard", timestamp: 0,
        axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => ({ pressed: false, touched: false, value: 0 })) })
    window.__pad = [mk(0), mk(1)]; navigator.getGamepads = () => window.__pad
})
const relayParam = E.RELAY_URL ? `&relay=${encodeURIComponent(E.RELAY_URL)}&name=${encodeURIComponent(E.NAME ?? STATION)}` : ""
const url = `${BASE}/?autojoin=${ROOM}&sphinx=${STATION}${E.FIELD === "1" ? "&field=1" : ""}&codesim=${encodeURIComponent(CODESIM)}${relayParam}`
await page.goto(url)
await page.waitForFunction(() => window.__sphinx, null, { timeout: 180000, polling: 1000 })
const status = await page.evaluate(() => window.__sphinx)
fs.mkdirSync(PROBE, { recursive: true })
fs.writeFileSync(PROBE + "/READY", JSON.stringify(status))
const done = new Set()
for (;;) {
    for (const f of fs.readdirSync(PROBE).filter(f => f.endsWith(".js"))) {
        if (done.has(f)) continue
        done.add(f)
        let out
        try { out = await page.evaluate(`(async () => { ${fs.readFileSync(PROBE + "/" + f, "utf8")} })()`) }
        catch (e) { out = { error: String(e).slice(0, 500) } }
        if (f.includes("shot")) await page.screenshot({ path: `${PROBE}/${f}.png` })
        fs.writeFileSync(`${PROBE}/${f}.out`, JSON.stringify(out ?? null))
    }
    if (fs.existsSync(PROBE + "/STOP")) break
    await new Promise(r => setTimeout(r, 300))
}
await browser.close()
