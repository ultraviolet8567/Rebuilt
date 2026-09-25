// One web address for the whole 8567 simulation.
//
//   node serve.mjs [siteDir] [--port 4000] [--relay 127.0.0.1:9002]
//
// * Serves the patched Synthesis build in siteDir (default ./site).
// * Maps /api/mira/... and /api/match_configs/... onto the build's Downloadables folder, which is
//   where the Synthesis dev server proxies them; a production build otherwise asks
//   synthesis.autodesk.com and cannot find the Sphinx model.
// * Passes WebSocket upgrades straight through to the glueball multiplayer relay, so players reach
//   the relay at the same address as the page (and through the same HTTPS tunnel).
//
// No dependencies beyond Node 18+.
import http from "node:http"
import net from "node:net"
import fs from "node:fs"
import path from "node:path"

const args = process.argv.slice(2)
const opt = (name, dflt) => {
    const i = args.indexOf(name)
    return i >= 0 ? args[i + 1] : dflt
}
const site = path.resolve(args.find(a => !a.startsWith("--") && !/^\d/.test(a) && !a.includes(":")) ?? "site")
const port = Number(opt("--port", 4000))
const [relayHost, relayPort] = opt("--relay", "127.0.0.1:9002").split(":")

const types = {
    ".html": "text/html; charset=utf-8", ".js": "text/javascript", ".mjs": "text/javascript",
    ".css": "text/css", ".json": "application/json", ".svg": "image/svg+xml", ".png": "image/png",
    ".webp": "image/webp", ".wasm": "application/wasm", ".zip": "application/zip",
    ".mira": "application/octet-stream", ".glb": "model/gltf-binary", ".txt": "text/plain",
}

function resolve(urlPath) {
    let p = decodeURIComponent(urlPath.split("?")[0])
    if (p.startsWith("/api/mira/") || p.startsWith("/api/match_configs/")) p = "/Downloadables" + p.slice(4)
    const file = path.join(site, p)
    if (!file.startsWith(site)) return null // no escaping the site folder
    if (fs.existsSync(file) && fs.statSync(file).isDirectory()) return path.join(file, "index.html")
    return file
}

const server = http.createServer((req, res) => {
    const file = resolve(req.url ?? "/")
    if (!file || !fs.existsSync(file)) {
        // Single-page app: unknown paths get the page itself.
        if (!req.url?.startsWith("/api/")) return send(path.join(site, "index.html"), res)
        res.writeHead(404).end("not found")
        return
    }
    send(file, res)
})

function send(file, res) {
    res.writeHead(200, {
        "Content-Type": types[path.extname(file)] ?? "application/octet-stream",
        "Cache-Control": file.endsWith("index.html") ? "no-cache" : "public, max-age=3600",
    })
    fs.createReadStream(file).pipe(res)
}

// WebSocket upgrade -> relay, byte for byte.
server.on("upgrade", (req, socket, head) => {
    const upstream = net.connect(Number(relayPort), relayHost, () => {
        let raw = `${req.method} ${req.url} HTTP/1.1\r\n`
        for (let i = 0; i < req.rawHeaders.length; i += 2) raw += `${req.rawHeaders[i]}: ${req.rawHeaders[i + 1]}\r\n`
        upstream.write(raw + "\r\n")
        if (head?.length) upstream.write(head)
        upstream.pipe(socket)
        socket.pipe(upstream)
    })
    const close = () => { socket.destroy(); upstream.destroy() }
    upstream.on("error", close)
    socket.on("error", close)
})

server.listen(port, () => {
    console.log(`8567 simulation site: http://localhost:${port}  (relay -> ${relayHost}:${relayPort}, files ${site})`)
})
