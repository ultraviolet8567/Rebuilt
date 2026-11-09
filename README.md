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
./gradlew test             # boot the simulated robot headless and check it behaves
./gradlew build            # compile, format-check and test
./gradlew deploy           # deploy to the roboRIO, exactly as before
```

Simulation-only files (`simgui.json`, `networktables.json`, `logs/`, `ctre_sim/`)
are gitignored.

## If you encounter problems

The Java compilation error cannot be resolved to a type (error code 16777218) in a FIRST
Robotics Competition (FRC) project means your IDE (VS Code or Eclipse) cannot find the
definition, library, or import for the class or object you are referencing.

There are four steps

1. Build the Robot Code:
    • Open the VS Code Command Palette (Ctrl + Shift + P or Cmd + Shift + P on Mac).
    • Type and select: WPILib: Build Robot Code. This forces Gradle to download, resolve, and extract all vendor dependencies and classes into the build path.
2. Refresh Gradle Dependencies:
    • Open the Command Palette (Ctrl + Shift + P) and select: Java: Clean Java Language Server Workspace or Gradle: Refresh Gradle Dependencies.
3. Verify Your Vendor Libraries (vendordeps):
    • Check your vendordeps/ folder in the project root to ensure .json files are intact.
    • If you recently added a new library, re-run WPILib: Manage Vendor Libraries -> Install new library.
4. Check Imports and Spelling:
    • Verify that the class name is spelled correctly (Java is case-sensitive).
    • Hover over the red squiggly error and use the IDE quick-fix option (Ctrl + . / Cmd + .) to add the missing import statement.
