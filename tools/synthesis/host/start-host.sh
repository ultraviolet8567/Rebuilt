#!/bin/bash
# Start a simulation match server on this computer.
#   ./start-host.sh --internet      opens a free Cloudflare quick tunnel (https) -- use this
#   ./start-host.sh                 home network only, plain http: browsers may block game
#                                   controllers there (keyboard still works)
#   ./start-host.sh --hostname match.example.org --tunnel NAME
#                                   optional: a named Cloudflare tunnel, so the address and the
#                                   player links are the same every session. Needs the one-time
#                                   set-up in ../README.md ("Fixed host address"). The two values
#                                   can also come from SIM_HOSTNAME and SIM_TUNNEL.
# Room code: 6 characters, digits and capital letters. Change it with ROOM=ABC123 ./start-host.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOM="${ROOM:-SPHINX}"
PORT="${PORT:-4000}"
[ -x "$HERE/bin/glueball" ] && [ -d "$HERE/site" ] || { echo "Run ./build-site.sh first."; exit 1; }

# Fixed address (optional). Both values are needed; flags win over SIM_HOSTNAME / SIM_TUNNEL.
FIXED_HOST="${SIM_HOSTNAME:-}"
FIXED_TUNNEL="${SIM_TUNNEL:-}"
prev=""
for a; do
    case "$prev" in
        --hostname) FIXED_HOST="$a" ;;
        --tunnel) FIXED_TUNNEL="$a" ;;
    esac
    case "$a" in
        --hostname=*) FIXED_HOST="${a#--hostname=}" ;;
        --tunnel=*) FIXED_TUNNEL="${a#--tunnel=}" ;;
    esac
    prev="$a"
done
# --internet, as before, asks for the quick tunnel and takes precedence.
[ "${1:-}" = "--internet" ] && { FIXED_HOST=""; FIXED_TUNNEL=""; }
if [ -n "$FIXED_HOST$FIXED_TUNNEL" ]; then
    [ -n "$FIXED_HOST" ] && [ -n "$FIXED_TUNNEL" ] || { echo "A fixed address needs both --hostname and --tunnel (see ../README.md, \"Fixed host address\")."; exit 1; }
    FIXED_HOST="${FIXED_HOST#https://}"; FIXED_HOST="${FIXED_HOST%/}"
    [[ "$FIXED_HOST" =~ ^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$ ]] || { echo "--hostname must be a name like match.example.org, not '$FIXED_HOST'."; exit 1; }
    command -v cloudflared >/dev/null || { echo "Install cloudflared first (brew install cloudflared)."; exit 1; }
fi

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
elif [ -n "$FIXED_HOST" ]; then
    # --url sends everything arriving on the tunnel to this server, so no config.yml is needed.
    cloudflared tunnel run --url "http://localhost:$PORT" "$FIXED_TUNNEL" > "$HERE/tunnel.log" 2>&1 &
    TUNNEL_PID=$!
    for _ in $(seq 1 30); do
        grep -q 'Registered tunnel connection' "$HERE/tunnel.log" && break
        kill -0 "$TUNNEL_PID" 2>/dev/null || { echo "The Cloudflare tunnel '$FIXED_TUNNEL' did not start; see $HERE/tunnel.log and ../README.md (\"Fixed host address\")."; exit 1; }
        sleep 1
    done
    grep -q 'Registered tunnel connection' "$HERE/tunnel.log" || echo "The tunnel has not reported a connection yet; if the links fail, see $HERE/tunnel.log."
    BASE="https://$FIXED_HOST"
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
