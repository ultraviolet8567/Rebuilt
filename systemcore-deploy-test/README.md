# Systemcore deploy test

A minimal WPILib 2027 robot program that shows a deploy to the DIY Systemcore (Raspberry Pi 5) works from this laptop. The program makes no motor, CAN or SmartIO objects, so it is safe with any wiring on the controller.

## What it does

- `Robot.java` publishes a build nonce, a counter, the Java version and `/etc/os-release` to the NetworkTables table `/DeployTest`. The counter runs in `robotPeriodic`, so no Driver Station is necessary.
- `NtProbe.java` runs on the laptop. It connects to the controller on port 5810, lists the topics, and compares `/DeployTest/nonce` with the nonce of the last deployed build.
- Each build makes a new nonce. A match shows that the program on the controller is the one that was deployed, and not an older one.

## Use

Requirements: OpenJDK 25 at `/opt/homebrew/opt/openjdk@25` (set in `gradle.properties`) and a network connection for the first build.

```bash
./gradlew deploy -ProbotAddress=169.254.99.210
```

Wait approximately 20 seconds. The `robot` service on the clone waits for CAN interfaces before it starts the program.

```bash
./gradlew ntProbe -ProbotAddress=169.254.99.210
```

The last line is `RESULT: PASS` when the controller runs this build and the counter advances.

`-ProbotAddress` is optional on this setup. Without it, GradleRIO found the Pi through its default host name and `ntProbe` connected to `robot.local` (both tested 2026-11-23 on a direct ethernet cable where both ends have link-local 169.254.x.x addresses). Give the address when the name does not resolve. To see what is on a controller before a deploy, add `-Pexpect=false` to the `ntProbe` command.

## From VS Code

Open this folder in VS Code. `.vscode/tasks.json` has four tasks (Terminal > Run Task):

- `Systemcore: deploy, then probe` is the default build task. Cmd+Shift+B runs it.
- `Systemcore: deploy` and `Systemcore: probe` run the two steps separately.
- `Systemcore: robot service log` shows the last lines of the program's output on the controller.

The tasks run the same Gradle commands as above. The WPILib VS Code extension is not installed on this Mac. Its "Deploy Robot Code" command also runs `./gradlew deploy`, so the result is the same. The extension comes with the WPILib installer; use the 2027 alpha-6 installer while the controller has image 11.

## With the FIRST Driver Station

The Driver Station does not deploy code. During a deploy, GradleRIO asks the Driver Station on `localhost:6770` for the address of the robot it is connected to, and uses that address. With the Driver Station connected, `./gradlew deploy` needs no address and no name lookup.

- Use Driver Station 2027.0.0-alpha-6 with image 11. The WPILib alpha-7 release note says Driver Station alpha-7 is necessary for alpha-7.
- After the Driver Station starts, press the spacebar once. It does not connect until that emergency-stop check is done.
- The team number field accepts an IP address. On the direct ethernet cable it is set to `169.254.99.210`, because the controller has no team number and no 10.TE.AM.2 address.
- Tested 2026-11-23: the deploy log showed `FIRST Driver Station reported IP: 169.254.99.210`, and `ntProbe` gave PASS.

## On a Windows laptop

Set up on 2026-11-23 on the team laptop `MSI`, connected to the controller's own Wi-Fi network `SYSTEMCORE` (controller at `172.30.0.1`).

- Tools: the WPILib 2027.0.0-alpha-6 Windows installer. Its files go to `C:\Users\Public\wpilib\2027_alpha5` (the installer for alpha-6 uses the year name `2027_alpha5`), with JDK 25 in the `jdk` folder and Gradle 9.4.1.
- Three project files are different on that laptop: `settings.gradle` has `wpilibYear = '2027_alpha5'`, the Gradle wrapper is 9.4.1, and `gradle.properties` gives the Windows JDK path.
- The first build needs a connection to the internet. The installer's offline library store does not have some parent POM files (for example for `gson`). After one online build, `--offline` builds work.
- `deploy.cmd` in the project folder on the laptop sets `JAVA_HOME`, deploys with `--offline`, waits, then runs `ntProbe`. Double-click it.
- Do not give `-ProbotAddress=172.30.0.1` to `deploy` with an older copy of `build.gradle`. That address is a GradleRIO default, and a second entry stopped the build. The current `build.gradle` ignores the duplicate.
- Result: deploy from the laptop completed and `ntProbe` gave PASS, also with `--offline` and no internet.

## WPILib version and OS image

The GradleRIO version in `build.gradle` must agree with the Systemcore OS image on the controller.

| OS image | GradleRIO / WPILib | Basis |
|---|---|---|
| 11 | 2027.0.0-alpha-6 | Measured here: alpha-6 runs, alpha-7 does not |
| 12, 13 | 2027.0.0-alpha-6 | Not tested. Assumed from the release dates (both are before alpha-7) |
| 14 or later | 2027.0.0-alpha-7 or later | Release notes. WPILib alpha-7: "Systemcore image 14 is required for this release." Image 14 and 15 titles: "REQUIRES WPILIB ALPHA 7". Not tested here |

Which of images 16 to 18 need alpha-8 is not stated in their release notes. Check before a choice of image for an alpha-8 project.

A program built with the wrong version deploys without an error, then stops at start. The `robot` service log shows `Error: MRC API version mismatch` and the service restarts in a loop.

To change to alpha-7, change the plugin version and the `gradleRio=` text in `writeBuildInfo`, add the `application` plugin, replace the `jar` block with `configureApplication` (see the comment in `build.gradle`), and change `Main.java` to `RobotBase.startRobot(Robot::new)`.

## Result on 2026-11-23

Controller: Raspberry Pi 5 clone, image `limelightosr-beta-11` (built 2026-06-14), Bobcat MCP2515 CAN setup, connected through a USB ethernet adapter at 169.254.99.210.

| Build | Deploy | Program |
|---|---|---|
| alpha-7 | Completed | Stopped at start: MRC API version mismatch |
| alpha-6 | Completed | Ran. `ntProbe` gave PASS (nonce matched, counter advanced, and the program was still up after 128 s) |

## Controller log without SSH

The web interface API gives the service state and the log:

```bash
curl -s "http://169.254.99.210:4802/api/logs/service/robot?lines=60"
```

```bash
curl -s "http://169.254.99.210:4802/api/systemd/services"
```
