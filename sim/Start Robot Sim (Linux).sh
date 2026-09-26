#!/bin/bash
# Start the 8567 robot program for the Synthesis simulation (Ubuntu and other Linux).
# In Files: right-click this file and choose "Run as a Program". Or in Terminal:
#   bash "sim/Start Robot Sim (Linux).sh"
# Leave the window open while you play; close it (or press Ctrl+C) to stop.

# Started from Files there is no window to show progress in, so reopen in a terminal.
if [ ! -t 1 ] && [ -z "${SIM_IN_TERMINAL:-}" ]; then
    export SIM_IN_TERMINAL=1
    for t in gnome-terminal ptyxis kgx konsole xfce4-terminal x-terminal-emulator xterm; do
        if command -v "$t" >/dev/null; then
            case "$t" in
                gnome-terminal|kgx) exec "$t" -- bash "$0" "$@" ;;
                ptyxis) exec "$t" --new-window -- bash "$0" "$@" ;;
                *) exec "$t" -e bash "$0" "$@" ;;
            esac
        fi
    done
fi

cd "$(dirname "$0")/.." || exit 1

# Java 17: the one WPILib installs, else a system Java 17 (sudo apt install openjdk-17-jdk).
unset JAVA_HOME
for j in "$HOME/wpilib/2026/jdk" "$HOME/wpilib/2027/jdk" /usr/lib/jvm/java-17-openjdk-* /usr/lib/jvm/temurin-17-*; do
    if [ -x "$j/bin/java" ] && "$j/bin/java" -version 2>&1 | grep -q 'version "17'; then
        export JAVA_HOME="$j"; break
    fi
done
if [ -z "$JAVA_HOME" ]; then
    echo "Could not find Java 17. Install WPILib 2026 (see the simulation guide), then try again."
    read -r -p "Press Return to close."; exit 1
fi

echo "Starting the robot program (the first run downloads files and takes a few minutes)..."
echo "When you see 'Robot program startup complete', open your match link in Chrome."
# bash, not ./gradlew: a ZIP download loses the file's run permission.
bash ./gradlew simulateJava -Psynthesis "$@"
read -r -p "The robot program stopped. Press Return to close."
