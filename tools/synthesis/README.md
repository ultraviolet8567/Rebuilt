# Synthesis multiplayer spike

Test of whether [Autodesk Synthesis](https://github.com/Autodesk/synthesis) can host a networked,
multiplayer match where each player's robot runs the real 8567 robot code. Tested 2026-09-25
against Synthesis `dev` @ `9f227a2`.

## Pieces

| Where | What |
|---|---|
| robot code (`-Psynthesis`) | `ModuleIOSynthesis`, `GyroIOSynthesis`, `util/SynthesisDevices` publish the motors, encoders and gyro over the HALSim websocket |
| `fission.patch` | two Synthesis changes: `?codesim=ws://host:port/wpilibws` picks the robot program per tab, and URDF import keeps joints named `dof_*` instead of welding them |
| `SwerveCodeSim.ts` | goes in `fission/src/dev/`; spawns Sphinx, connects robot-code devices to its wheels, steering, intake pivot, hood and gyro, calibrates hinge directions, forwards controllers |
| `urdf/` | builds `urdf/out/sphinx_urdf.zip` (copy to `fission/public/`) from the Onshape CAD plus `DriveConstants` |
| `sphinx_test.mjs`, `sphinx_session.mjs` | headless Chrome players: a one-shot drive/intake/hood test, and a long-lived session that runs probe snippets |
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
`startControllerForwarding()` so the player's controllers reach the robot code. The tab must be
visible while it attaches: it nudges each hinge to learn its direction, and a hidden tab runs no
physics.

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

## Player links

`start-host.sh` (in `host/`) prints one link per station. Parameters:

| Parameter | Meaning |
|---|---|
| `autojoin=ROOM` | relay room (must already exist: glueball `-r ROOM`) |
| `sphinx=red2` | station; spawns Sphinx there and runs the whole setup |
| `relay=ws(s)://host:port` | relay address (sets the Synthesis multiplayer preferences) |
| `field=1` | spawn the field if nobody has; the first player in keeps score |
| `codesim=ws://localhost:3300/wpilibws` | this player's robot program (default port 3300) |
| `view=driver\|station\|follow` | starting camera (default `driver`); V cycles it in play |

Camera views use the field's own camera points ("Red Alliance 1" ... "Blue Alliance 3"; the 2026
field ships them at 1.5 m eye height behind each station). Station 1 is the drivers' left.

Any number of players from 1 to 6 works: empty stations just have no robot. Checked 2026-09-26
with `count_test.mjs` (1, 2, 3 and 5 robots; 6 was checked 2026-09-25): every client saw every
robot, each camera sat at its own station, and all robots went auto -> teleop -> endgame ->
ended together.

## Known gaps

- Keep the browser tab landscape. In portrait Synthesis shows an overlay, and a drive test run
  that way moved the robot 0.15 m where the same command in landscape moved it 1.67 m.

## Sphinx model (URDF)

`urdf/build_sphinx_urdf.py` reads the "Rebuilt Robot Final" Final Assembly from Onshape (read only)
and writes `urdf/out/sphinx_urdf.zip`:

- Geometry: one glTF export of the assembly (`urdf/export_gltf.py`, 6 API calls), split into
  links and decimated to ~175k triangles.
- `dof_intake_pivot`: the arm is found by cutting the Onshape mates on the 27.5" pivot shaft's
  axis; whatever falls away from the frame moves. Joint 0 is **stowed** (robot code 0.1 rad), as the
  robot starts a match; the absolute encoder aliases anywhere else.
- `dof_hood`: the hood group (Hood Gear Bracket and its rollers) about the flywheel axis.
- Swerve: the CAD modules are welded solid, so four modules are generated from `DriveConstants`
  (the CAD module spacing, 0.55 m, matches it).

Credentials: `ONSHAPE_KEYS_FILE=<file with URDF_ONSHAPE_ACCESS_KEY / URDF_ONSHAPE_SECRET_KEY>`.
Responses are cached in `urdf/.cache/` (git-ignored), so rebuilding costs no API calls until the
CAD changes. `urdf/survey.py` lists every moving mate if the CAD is restructured.

```bash
uv run --with requests --with numpy --with pygltflib --with trimesh --with fast-simplification --with scipy python build_sphinx_urdf.py
```

Measured 2026-09-25 with the robot code driving Sphinx through the controllers: forward 1.73 m
straight, reverse 1.92 m straight, rotate +120 deg, intake deploy/middle/stow reached their code
targets, hood 0 -> 0.29 -> 0. Two players, both Sphinx: each saw the other's intake move, and a
collision pushed player 2's robot 0.29 m, identically in both views.

Known gaps: strafing yaws the robot (-36 deg over 0.6 m); deploying the intake shoves the chassis;
Synthesis imports a URDF hinge's direction inconsistently between loads (hence calibration).
