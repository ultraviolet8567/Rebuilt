#!/bin/bash
# Double-click to start the 8567 robot program for the Synthesis simulation.
# Leave this window open while you play; close it (or press Ctrl+C) to stop.
cd "$(dirname "$0")/.." || exit 1

# Java 17: the one WPILib installs, else any Java 17 on this Mac. Check the version itself:
# java_home prints the default Java (often 8) when it cannot find the one asked for.
unset JAVA_HOME
for j in "$HOME/wpilib/2026/jdk" "$HOME/wpilib/2027/jdk" "$(/usr/libexec/java_home -v 17 2>/dev/null)" \
         /opt/homebrew/opt/openjdk@17 /usr/local/opt/openjdk@17; do
    if [ -n "$j" ] && [ -x "$j/bin/java" ] && "$j/bin/java" -version 2>&1 | grep -q 'version "17'; then
        export JAVA_HOME="$j"; break
    fi
done
if [ -z "$JAVA_HOME" ]; then
    echo "Could not find Java 17. Install WPILib 2026 (see the simulation guide), then try again."
    read -r -p "Press Return to close."; exit 1
fi

echo "Starting the robot program (the first run downloads files and takes a few minutes)..."
echo "When you see 'Robot program startup complete', open your match link in Chrome."
./gradlew simulateJava -Psynthesis "$@"
read -r -p "The robot program stopped. Press Return to close."
