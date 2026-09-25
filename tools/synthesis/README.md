# Synthesis multiplayer spike

Test of whether [Autodesk Synthesis](https://github.com/Autodesk/synthesis) can host a networked,
multiplayer match where each player's robot runs the real 8567 robot code. Tested 2026-09-25
against Synthesis `dev` @ `9f227a2`.

## Pieces

| Where | What |
|---|---|
| robot code (`-Psynthesis`) | `ModuleIOSynthesis`, `GyroIOSynthesis`, `util/SynthesisDevices` publish the motors, encoders and gyro over the HALSim websocket |
| `fission-codesim-url.patch` | lets a browser tab use `?codesim=ws://host:port/wpilibws` instead of the hard-coded `localhost:3300` |
| `SwerveCodeSim.ts` | goes in `fission/src/dev/`; connects robot-code devices to a swerve model's wheels, hinges and chassis gyro, plus setup helpers |
| `nt_sampler.py` | records robot signals over NetworkTables at 10 Hz (`uv run --with pyntcore python nt_sampler.py <secs> out.jsonl`); used to check every controller binding |
| `player2.mjs` | goes in `fission/`; a headless Chrome player used to run a second client in the test |
| glueball | Synthesis's relay server (`glueball -h -p 9002 -r RBLT26`), run on a separate machine |

## Running one player

```bash
./gradlew simulateJava -Psynthesis                          # player 1 robot code, ws :3300
./gradlew simulateJava -Psynthesis -PsimPort=3301 -PntPort=5811   # a second copy on the same computer
```

Open `http://localhost:3000/?autojoin=RBLT26&codesim=ws://localhost:3300/wpilibws` with the
Synthesis preferences `MultiplayerHost`/`MultiplayerPort` pointing at the relay, then in the dev
console: `(await import('/src/dev/SwerveCodeSim.ts')).setupPlayer(true, [6, 0.1, -2.5])`
(pass `false` for every player after the first, who has already spawned the field), then
`startControllerForwarding()` so the player's controllers reach the robot code.

Synthesis only forwards driver-station and joystick changes to the robot program when it also
sends `>new_data`. Synthesis's `SimDriverStation.setMode` does not send it, so in this test the
robot stayed disabled until the helpers sent it with each update.

## Controllers

Synthesis does not pass a physical gamepad to WPILib robot code (its `SimGamepadInput` is never
created and uses the FTC format). `startControllerForwarding()` reads the browser Gamepad API:
first controller -> port 0 (driver), second -> port 1 (operator), in WPILib Xbox layout. With no
controller, the keyboard drives: WASD translate, arrow keys turn. Rumble is not passed back.

Every binding in `RobotContainer.configureBindings()` was exercised this way on 2026-09-25 and
produced the expected robot response (drive, slow mode, X-lock, heading reset, aim-at-hub, funnel,
indexer, hood trim, both shots with the kicker interlock, pivot positions, shake).

## Known gaps

- Odometry is not seeded from where Synthesis places the robot, so field-position features
  (aim-at-hub target, ranged-shot distance) use a wrong pose.
- Shooter, intake and indexer run on the robot's local models; no fuel leaves the robot.
- Keep the browser tab landscape. In portrait Synthesis shows an overlay, and a drive test run
  that way moved the robot 0.15 m where the same command in landscape moved it 1.67 m.
