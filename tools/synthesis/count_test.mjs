// Matches with 1 to 6 robots: starts that many scripted players (player.mjs) against robot
// programs already running on ports 3300..3305, then plays a short match and checks it.
//
//   node count_test.mjs red2  red1,blue3  red1,red2,red3 ...
//
// Env: BASE (site, default http://localhost:3000), RELAY, RELAY_PORT, ROOM, OUT (work dir).
// Run from a folder where `playwright` resolves (player.mjs has to sit next to it too).
import { spawn } from "node:child_process"
import fs from "node:fs"
import path from "node:path"

const E = process.env
const OUT = E.OUT ?? "count_test_out"
const AUTO = +(E.AUTO ?? 4), TELEOP = +(E.TELEOP ?? 6)
const sleep = ms => new Promise(r => setTimeout(r, ms))

function probe(dir, name, body, timeoutMs = 60000) {
    fs.writeFileSync(path.join(dir, name + ".js"), body)
    const out = path.join(dir, name + ".js.out")
    const t0 = Date.now()
    return (async () => {
        while (!fs.existsSync(out)) {
            if (Date.now() - t0 > timeoutMs) return { error: "probe timeout" }
            await sleep(200)
        }
        return JSON.parse(fs.readFileSync(out, "utf8"))
    })()
}

// Positions of every robot this client sees, and which one is its own.
const LOOK = `
const W = window.__W, mine = window.__X.myRobot(), robots = []
for (const so of W.sceneRenderer.mirabufSceneObjects.getAll()) {
    if (so.miraType !== 1) continue // robots only (MiraType.ROBOT)
    const id = so.mechanism?.nodeToBody.get(so.rootNodeId); if (!id) continue
    const p = W.physicsSystem.getBody(id).GetPosition()
    robots.push({ mine: so === mine, x: +p.GetX().toFixed(2), z: +p.GetZ().toFixed(2), alliance: so.alliance, station: so.station })
}
const MM = (await import("/src/systems/match_mode/MatchMode.ts")).default.getInstance()
const c = W.sceneRenderer.mainCamera
return { mode: MM.getMatchModeType(), robots, cam: [c.position.x, c.position.y, c.position.z].map(v => +v.toFixed(2)),
    controls: W.sceneRenderer.currentCameraControls.constructor.name, fps: W.sceneRenderer.fps ?? null }`

async function scenario(stations) {
    const tag = stations.join("+")
    console.log(`\n=== ${stations.length} robot(s): ${tag}`)
    const players = []
    for (const [i, st] of stations.entries()) {
        const dir = path.resolve(OUT, `${tag}_${st}`)
        fs.rmSync(dir, { recursive: true, force: true })
        fs.mkdirSync(dir, { recursive: true })
        const child = spawn("node", ["player.mjs"], {
            env: { ...E, STATION: st, FIELD: "1", PROBE_DIR: dir, CODESIM: `ws://localhost:${3300 + i}/wpilibws` },
            stdio: ["ignore", "ignore", "pipe"],
        })
        child.stderr.on("data", d => fs.appendFileSync(path.join(dir, "stderr.log"), d))
        players.push({ st, dir, child })
        await sleep(i === 0 ? 20000 : 4000) // the first player brings the field
    }
    const t0 = Date.now()
    for (const p of players) {
        while (!fs.existsSync(path.join(p.dir, "READY")) && Date.now() - t0 < 300000) await sleep(1000)
        const r = fs.existsSync(path.join(p.dir, "READY")) ? JSON.parse(fs.readFileSync(path.join(p.dir, "READY"), "utf8")) : { ready: false, error: "no READY" }
        p.ready = r.ready
        console.log(`  ${p.st}: ${r.ready ? "ready" : "NOT READY " + (r.error ?? "")}`)
    }
    await sleep(3000)
    const before = await Promise.all(players.map(p => probe(p.dir, "before", LOOK)))
    const FPS = "return await new Promise(r => { let n = 0; const t = performance.now(); const f = () => { n++; performance.now() - t < 2000 ? requestAnimationFrame(f) : r(n / 2) }; requestAnimationFrame(f) })"
    const fps = await Promise.all(players.map(p => probe(p.dir, "fps", FPS)))
    console.log(`  frames per second: ${fps.join(", ")}`)
    players.forEach((p, i) => {
        const b = before[i]
        const mine = b.robots?.find(r => r.mine)
        console.log(`  ${p.st} sees ${b.robots?.length} robot(s); camera ${b.controls} at ${JSON.stringify(b.cam)}; own robot ${mine ? `${mine.alliance}${mine.station} at (${mine.x}, ${mine.z})` : "?"}`)
    })

    // Short match started by the first player (what the Start Match button does).
    await probe(players[0].dir, "start", `await window.__X.startMatch({ autonomousTime: ${AUTO}, teleopTime: ${TELEOP}, endgameTime: 2 }); return true`)
    const timeline = players.map(() => [])
    const t1 = Date.now()
    let n = 0
    while (Date.now() - t1 < (AUTO + TELEOP) * 2000 + 4000) {
        // In teleop, the last player pushes forward on the left stick.
        const s = await Promise.all(players.map(p => probe(p.dir, `t${n}`, LOOK, 10000)))
        const drive = players.at(-1)
        const inTeleop = s.at(-1)?.mode === "Teleop" || s.at(-1)?.mode === "Endgame"
        await probe(drive.dir, `pad${n}`, `window.__pad[0].axes[1] = ${inTeleop ? -1 : 0}; return true`, 10000)
        s.forEach((x, i) => timeline[i].push({ t: +((Date.now() - t1) / 1000).toFixed(1), ...x }))
        n++
        await sleep(700)
    }
    await probe(players.at(-1).dir, "padstop", "window.__pad[0].axes[1] = 0; return true")

    const report = players.map((p, i) => {
        const tl = timeline[i]
        const modes = [...new Set(tl.map(x => x.mode))]
        const own = x => x.robots?.find(r => r.mine)
        const at = m => tl.filter(x => x.mode === m).map(own).filter(Boolean)
        const moved = a => (a.length > 1 ? Math.hypot(a.at(-1).x - a[0].x, a.at(-1).z - a[0].z) : 0)
        return {
            station: p.st,
            modes,
            robotsSeen: [...new Set(tl.map(x => x.robots?.length))],
            autoMove: +moved(at("Autonomous")).toFixed(2),
            teleopMove: +moved([...at("Teleop"), ...at("Endgame")]).toFixed(2),
            startPos: at("Autonomous")[0],
            endMode: tl.at(-1)?.mode,
        }
    })
    for (const r of report) console.log("  " + JSON.stringify(r))
    fs.writeFileSync(path.join(OUT, `${tag}.json`), JSON.stringify({ before, timeline, report }, null, 1))

    for (const p of players) fs.writeFileSync(path.join(p.dir, "STOP"), "")
    await sleep(4000)
    for (const p of players) p.child.kill()
    await sleep(3000)
    return report
}

fs.mkdirSync(OUT, { recursive: true })
for (const arg of process.argv.slice(2)) await scenario(arg.split(","))
process.exit(0)
