#!/bin/bash
# Start a simulation match server on this computer.
#   ./start-host.sh --internet      opens a free Cloudflare quick tunnel (https) -- use this
#   ./start-host.sh                 home network only, plain http: browsers may block game
#                                   controllers there (keyboard still works)
# Room code: 6 characters, digits and capital letters. Change it with ROOM=ABC123 ./start-host.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOM="${ROOM:-SPHINX}"
PORT="${PORT:-4000}"
[ -x "$HERE/bin/glueball" ] && [ -d "$HERE/site" ] || { echo "Run ./build-site.sh first."; exit 1; }

cleanup() { kill $(jobs -p) 2>/dev/null || true; }
trap cleanup EXIT

"$HERE/bin/glueball" -h -p 9002 -r "$ROOM" > "$HERE/relay.log" 2>&1 &
node "$HERE/serve.mjs" "$HERE/site" --port "$PORT" --relay 127.0.0.1:9002 &
sleep 1

BASE="http://$(ipconfig getifaddr en0 2>/dev/null || hostname -I 2>/dev/null | awk '{print $1}'):$PORT"
if [ "${1:-}" = "--internet" ]; then
    command -v cloudflared >/dev/null || { echo "Install cloudflared first (brew install cloudflared)."; exit 1; }
    cloudflared tunnel --url "http://localhost:$PORT" > "$HERE/tunnel.log" 2>&1 &
    for _ in $(seq 1 30); do
        # No match yet is normal while the tunnel starts; don't let set -e treat it as a failure.
        BASE="$(grep -o 'https://[a-z0-9-]*\.trycloudflare\.com' "$HERE/tunnel.log" | head -1 || true)"; [ -n "$BASE" ] && break; sleep 1
    done
    [ -n "$BASE" ] && [[ "$BASE" == https://* ]] || { echo "The Cloudflare tunnel did not start; see $HERE/tunnel.log."; exit 1; }
fi
RELAY="${BASE/http/ws}"

echo
echo "Room $ROOM is open. Send each player their own link (they must run the robot program first):"
for st in red1 red2 red3 blue1 blue2 blue3; do
    echo "  $st: $BASE/?autojoin=$ROOM&sphinx=$st&relay=$RELAY&field=1"
done
echo
echo "The first player in brings the field and keeps score, so the host (or whoever will stay"
echo "longest) should join first. Leave this window open; Ctrl+C ends the session."
wait
