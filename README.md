# Rebuilt (2026) — branch `sim/shooter-seperate-sim`

This branch starts from the team's most recent competition code (`shooter-seperate`),
merges the review fixes from `bugfix/shooter-seperate-fixes`, fixes several more
problems, and adds full **desktop simulation** so the robot program can be run,
driven and watched in AdvantageScope on a laptop with no robot present.

## Documents

- **[docs/simulator-guide.html](docs/simulator-guide.html)** — how to install, run and
  use the simulator. Written for non-programmers; includes tooltips and links to
  learning resources.
- **[docs/sim-branch-changes.html](docs/sim-branch-changes.html)** — every change on
  this branch and why, the simulation architecture, what the tests prove, and what
  to verify on the real robot.

(Open the HTML files in a browser; GitHub shows them as source.)

## Quick start

```
./gradlew simulateJava     # run the simulator (needs Java 17; WPILib VS Code has it)
./gradlew test             # 35 checks: boots the simulated robot headless and verifies it behaves
./gradlew build            # compile, format-check and test
./gradlew deploy           # deploy to the roboRIO, exactly as before
```

Simulation-only files (`simgui.json`, `networktables.json`, `logs/`, `ctre_sim/`)
are gitignored.
