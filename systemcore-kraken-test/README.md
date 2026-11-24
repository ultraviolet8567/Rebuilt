# Systemcore Kraken test

A WPILib 2027 robot program that turns one Kraken (TalonFX) on the DIY Systemcore (Raspberry Pi 5) at a low open-loop voltage. It is a copy of `systemcore-deploy-test` with Phoenix 6 added. Use `systemcore-deploy-test` when a program that moves nothing is necessary.

**WARNING: this program turns a motor when the Driver Station is enabled in Teleoperated mode. Clamp the motor and keep the shaft clear before Enable.**

## What it does

- Makes a `TalonFX` with CAN ID 0 on bus 1 (`can_s1`). On the 2-channel HAT, `can_s1` is the connector with the label CAN0.
- While enabled in Teleoperated mode, it commands the voltage in `/KrakenTest/commandVolts`. The start value is 1.0 V and the limit is 2.0 V in each direction. A free Kraken X60 turns at approximately 500 rpm per volt.
- When disabled, it commands neutral output.
- It sets a stator current limit of 20 A. It writes only the current-limit group, so other settings on the motor do not change.
- It publishes the motor's connection state, firmware version, supply voltage, motor voltage, stator current, speed and temperature to `/KrakenTest`.

No external encoder is necessary. Open-loop voltage control uses no feedback, and the speed in `/KrakenTest/rotorRpm` comes from the sensor inside the motor.

## Use

```bash
./gradlew deploy -ProbotAddress=169.254.99.210
```

Wait approximately 25 seconds, then read the motor signals:

```bash
./gradlew ntProbe -ProbotAddress=169.254.99.210
```

Then, on the Driver Station: select Teleoperated and click Enable. Click Disable to stop. Space and Enter are emergency stop.

To change the voltage, change `/KrakenTest/commandVolts` from a NetworkTables client (for example AdvantageScope or Elastic). To change the CAN ID, bus or limits, change the constants at the top of `Robot.java`.

## Versions

| Item | Version | Basis |
|---|---|---|
| GradleRIO / WPILib | 2027.0.0-alpha-6 | Necessary for OS image 11 (measured, see `systemcore-deploy-test/README.md`) |
| Phoenix 6 | 26.50.0-alpha-1 | The CTRE release for WPILib alpha-6. 26.70.0-alpha-2 is in the alpha-7 project |
| Kraken firmware | 26.99.0.104 | Read from the motor on 2026-11-24 |

The first build needs a connection to the internet to get Phoenix 6 from the CTRE server.

## On the Windows laptop

The project is in `C:\frc2027\systemcore-kraken-test` on the team laptop `MSI`, with the same three changed files as the deploy test (`settings.gradle`, the Gradle wrapper, `gradle.properties`) and a `deploy.cmd`. Phoenix 6 was downloaded with one online `deploy` and one online `ntProbe`. After that, `deploy --offline` and `ntProbe --offline` gave PASS on the `SYSTEMCORE` Wi-Fi (2026-11-24).

## Result on 2026-11-24

Deployed from the Mac on the direct ethernet cable. `ntProbe` gave PASS. With the robot disabled, the program reported: connected = true, supply 12.4 V, current limit configuration OK, 0 rpm. The CAN bus also has a REV PDH with ID 1.

The motor turned when FIRST Driver Station 2027.0.0-alpha-6 on the Windows laptop (on the `SYSTEMCORE` Wi-Fi) was enabled in Teleoperated mode, as seen at the bench. The speed during the run was not recorded: the program log has one line each 5 seconds and no line fell in the run.

The NI 2026 Driver Station must be closed. When both Driver Stations ran on the laptop, the 2027 one did not connect to the controller.
