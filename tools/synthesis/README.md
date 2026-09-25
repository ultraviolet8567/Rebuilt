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
(pass `false` for every player after the first, who has already spawned the field).

Synthesis only forwards driver-station and joystick changes to the robot program when it also
sends `>new_data`. Synthesis's `SimDriverStation.setMode` does not send it, so in this test the
robot stayed disabled until the helpers sent it with each update.
