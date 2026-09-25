/**
 * Spike helper: drive a swerve robot in Synthesis from real WPILib robot code.
 *
 * Synthesis's wiring graph requires identical unit types on both ends of a connection, and a CAN
 * motor's output is typed as a linear "Position" while a steering hinge takes an "Angle", so a
 * motor cannot be wired to a hinge in the UI. This module builds the flows directly instead:
 *
 *   CANMotor Drive[n] -> wheel n       CANEncoder Drive[n] <- wheel n rotation
 *   CANMotor Turn[m]  -> hinge m       CANEncoder Turn[m]  <- hinge m angle
 *   Gyro Pigeon2[30]  <- chassis yaw
 *
 * Module order matches the robot code: 0 front-left, 1 front-right, 2 back-left, 3 back-right.
 */
import JOLT from "@/util/loading/JoltSyncLoader"
import World from "@/systems/World"
import type MirabufSceneObject from "@/mirabuf/MirabufSceneObject"
import WPILibBrain from "@/systems/simulation/wpilib_brain/WPILibBrain"
import WheelDriver from "@/systems/simulation/driver/WheelDriver"
import HingeDriver from "@/systems/simulation/driver/HingeDriver"
import type Stimulus from "@/systems/simulation/stimulus/Stimulus"
import GyroStimulus from "@/systems/simulation/stimulus/GyroStimulus"
import { StimulusType } from "@/systems/simulation/stimulus/Stimulus"
import SimCANMotor from "@/systems/simulation/wpilib_brain/sim/SimCANMotor"
import SimCANEncoder from "@/systems/simulation/wpilib_brain/sim/SimCANEncoder"
import SimGyro from "@/systems/simulation/wpilib_brain/sim/SimGyro"

export type SwerveCodeSimOptions = {
    driveIds: number[] // CAN ids, module order FL FR BL BR
    turnIds: number[]
    gyro: string // SimDevice name as Synthesis lists it, e.g. "Pigeon2[30]"
    wheelMaxRadPerSec: number // must equal the robot code's full-output wheel speed
    robotWheelRadius: number // metres; the robot code's wheel, which may differ from the model's
    // Steering hinge signs. Leave undefined to calibrate at attach time: the same model has come
    // in with opposite hinge senses on different loads, so fixed signs are not reliable.
    steerCmdSign?: number
    steerEncSign?: number
    // Steering hinge speed at full output. The robot code's steering P loop runs through a 20 ms
    // loop plus a websocket round trip; at a URDF's default 30 rad/s it oscillated +-3 rad every
    // 100 ms. SwerveSimple's model carried pi rad/s.
    steerMaxRadPerSec?: number
    // Other motor-driven hinges, found by URDF joint name. maxRadPerSec is the joint speed at full
    // motor output (motor free speed / reduction); the robot code's IO sends volts / 12.
    // positive: which way the robot code's angle increases, "down" or "up" at the link's centre
    // of mass. Signs are found by calibration at attach time (see calibrateHinge).
    mechanisms?: { joint: string; motor: string; maxRadPerSec: number; positive: "down" | "up" }[]
}

export const SWERVE_SIMPLE_SIGNS = { steerCmdSign: 1, steerEncSign: 1 }

export const DEFAULT_8567: SwerveCodeSimOptions = {
    driveIds: [10, 11, 12, 13],
    turnIds: [20, 21, 22, 23],
    gyro: "Pigeon2[30]",
    wheelMaxRadPerSec: 102.8,
    robotWheelRadius: 0.04863,
    steerMaxRadPerSec: 6,
    // PivotIOSynthesis: angle increases toward deployed (down). HoodIOSynthesis: increases up.
    mechanisms: [
        { joint: "dof_intake_pivot", motor: "Pivot[5]", maxRadPerSec: 5.28, positive: "down" }, // NEO / (45 * 40/16)
        { joint: "dof_hood", motor: "Hood[4]", maxRadPerSec: 1.42, positive: "up" }, // NEO / (25 * 168/10)
    ],
}

const MODULE_TAGS = ["fl", "fr", "bl", "br"]
// biome-ignore lint/suspicious/noExplicitAny: drivers carry their joint's mirabuf info
const jointName = (d: any): string => d.info?.name ?? ""

type Vec = { x: number; y: number; z: number }

function wheelWorldPos(w: WheelDriver): Vec {
    const forward = new JOLT.Vec3(1, 0, 0)
    const up = new JOLT.Vec3(0, 1, 0)
    const t = w.constraint.GetWheelWorldTransform(0, forward, up).GetTranslation()
    const p = { x: t.GetX(), y: t.GetY(), z: t.GetZ() }
    JOLT.destroy(forward)
    JOLT.destroy(up)
    return p
}

/** Chassis-local: forward = +z, up = +y, so +x is the robot's LEFT (right-handed, y-up). */
function toChassis(robot: MirabufSceneObject, p: Vec): Vec {
    const body = World.physicsSystem.getBody(robot.getRootNodeId()!)!
    const c = body.GetCenterOfMassPosition()
    const r = body.GetRotation()
    const [qx, qy, qz, qw] = [-r.GetX(), -r.GetY(), -r.GetZ(), r.GetW()] // inverse rotation
    const v = { x: p.x - c.GetX(), y: p.y - c.GetY(), z: p.z - c.GetZ() }
    // v' = q v q*
    const ix = qw * v.x + qy * v.z - qz * v.y
    const iy = qw * v.y + qz * v.x - qx * v.z
    const iz = qw * v.z + qx * v.y - qy * v.x
    const iw = -qx * v.x - qy * v.y - qz * v.z
    return {
        x: ix * qw + iw * -qx + iy * -qz - iz * -qy,
        y: iy * qw + iw * -qy + iz * -qx - ix * -qz,
        z: iz * qw + iw * -qz + ix * -qy - iy * -qx,
    }
}

/** 0 FL, 1 FR, 2 BL, 3 BR from a chassis-local position. */
function moduleIndex(p: Vec): number {
    const front = p.z > 0
    const left = p.x > 0
    return (front ? 0 : 2) + (left ? 0 : 1)
}

/**
 * Nudge one hinge each way with nothing else driving it and report the output and angle signs
 * that make "positive" mean `wanted`: measure(before, after) > 0 when the joint moved that way.
 * Needs the physics running (a visible tab).
 */
async function calibrateHinge(h: HingeDriver, measure: () => number, maxVel: number) {
    const trials: { u: number; dq: number; dm: number }[] = []
    for (const u of [0.4, -0.4]) {
        await sleep(150)
        const q0 = h.constraint.GetCurrentAngle()
        const m0 = measure()
        h.accelerationDirection = u * Math.min(1, 3 / maxVel)
        await sleep(300)
        h.accelerationDirection = 0
        await sleep(150)
        trials.push({ u, dq: h.constraint.GetCurrentAngle() - q0, dm: measure() - m0 })
    }
    // Use whichever push actually moved the joint (one may be against a limit).
    const t = trials.reduce((a, b) => (Math.abs(b.dq) > Math.abs(a.dq) ? b : a))
    if (Math.abs(t.dq) < 0.02) return { cmd: 1, enc: 1, ok: false, trials }
    const movedWanted = t.dm > 0
    const cmd = Math.sign(t.u) * (movedWanted ? 1 : -1)
    const enc = Math.sign(t.dq) * (movedWanted ? 1 : -1)
    return { cmd, enc, ok: true, trials }
}

function yawAboutUp(x: number, z: number) {
    return Math.atan2(-z, x) // rotation about +Y, counter-clockwise seen from above
}

function wrap(a: number) {
    return Math.atan2(Math.sin(a), Math.cos(a))
}

export async function attachSwerveCodeSim(robot: MirabufSceneObject, opts: SwerveCodeSimOptions = DEFAULT_8567) {
    const layer = World.simulationSystem.getSimulationLayer(robot.mechanism)!
    if (!(robot.brain instanceof WPILibBrain)) robot.brain = new WPILibBrain(robot, "wpilib")
    const brain = robot.brain as WPILibBrain

    const wheels = layer.drivers.filter((d): d is WheelDriver => d instanceof WheelDriver)
    const mechNames = new Set((opts.mechanisms ?? []).map(m => m.joint))
    const hinges = layer.drivers.filter(
        (d): d is HingeDriver => d instanceof HingeDriver && !mechNames.has(jointName(d))
    )
    if (wheels.length !== 4 || hinges.length !== 4)
        throw new Error(`need 4 wheels + 4 steering hinges, found ${wheels.length} + ${hinges.length}`)

    // Modules by joint name (dof_fl_steer / dof_fl_wheel) when the model names them, otherwise by
    // where the wheel sits on the chassis.
    const report: Record<string, unknown>[] = []
    const byModule: { wheel?: WheelDriver; hinge?: HingeDriver }[] = [{}, {}, {}, {}]
    const tagIndex = (d: unknown) => MODULE_TAGS.findIndex(t => jointName(d).startsWith(`dof_${t}_`))
    wheels.forEach(w => {
        const local = toChassis(robot, wheelWorldPos(w))
        const i = tagIndex(w) >= 0 ? tagIndex(w) : moduleIndex(local)
        byModule[i].wheel = w
        report.push({ module: i, wheel: jointName(w) || w.idStr, local })
    })
    hinges.forEach(h => {
        const a = h.worldAnchor
        const local = toChassis(robot, { x: a.GetX(), y: a.GetY(), z: a.GetZ() })
        const i = tagIndex(h) >= 0 ? tagIndex(h) : moduleIndex(local)
        byModule[i].hinge = h
        report.push({ module: i, hinge: jointName(h) || h.displayName(), local })
    })

    const scaled = (stim: Stimulus, k: number) => ({
        supplierType: stim.supplierType,
        getSupplierValue: () =>
            // biome-ignore lint/suspicious/noExplicitAny: [position, velocity] pair
            (stim.getSupplierValue() as any[]).map(v => ({ ...v, value: k * v.value })),
    })
    const stimulusFor = (guid: string) =>
        layer.stimuli.find(s => s.id.guid === guid && s.id.type === StimulusType.STIM_ENCODER)

    // Calibrate with no flows attached, so nothing else is driving the joints.
    brain.loadSimConfig()
    // biome-ignore lint/suspicious/noExplicitAny: spike clears the brain's flow list
    ;(brain as any)._simFlows = []
    // biome-ignore lint/suspicious/noExplicitAny: every Synthesis driver has this field
    layer.drivers.forEach(d => ((d as any).accelerationDirection = 0))
    const body = World.physicsSystem.getBody(robot.getRootNodeId()!)!
    const chassisYaw = () => {
        const q = body.GetRotation()
        return Math.atan2(2 * (q.GetW() * q.GetY() + q.GetX() * q.GetZ()), 1 - 2 * (q.GetY() ** 2 + q.GetZ() ** 2))
    }
    const wheelYaw = (w: WheelDriver) => {
        const f = new JOLT.Vec3(1, 0, 0)
        const u = new JOLT.Vec3(0, 1, 0)
        const ax = w.constraint.GetWheelWorldTransform(0, f, u).GetAxisX()
        const y = yawAboutUp(ax.GetX(), ax.GetZ())
        JOLT.destroy(f)
        JOLT.destroy(u)
        return y
    }
    const cals: Awaited<ReturnType<typeof calibrateHinge>>[] = []
    for (const m of byModule) {
        if (!m.wheel || !m.hinge) throw new Error(`module missing a wheel or hinge: ${JSON.stringify(report)}`)
        m.hinge.setContinuousRotation()
        const w = m.wheel
        let last = wrap(wheelYaw(w) - chassisYaw())
        let acc = 0 // unwrapped counter-clockwise rotation of the wheel relative to the chassis
        const measure = () => {
            const now = wrap(wheelYaw(w) - chassisYaw())
            acc += wrap(now - last)
            last = now
            return acc
        }
        cals.push(opts.steerCmdSign !== undefined ? { cmd: 1, enc: 1, ok: true, trials: [] } : await calibrateHinge(m.hinge, measure, opts.steerMaxRadPerSec ?? m.hinge.maxVelocity))
    }

    byModule.forEach((m, i) => {
        if (!m.wheel || !m.hinge) throw new Error(`module ${i} is missing a wheel or hinge: ${JSON.stringify(report)}`)
        const driveDev = `Drive[${opts.driveIds[i]}]`
        const turnDev = `Turn[${opts.turnIds[i]}]`

        // The model's wheel may not be the robot's wheel. Scale so that a given motor output gives
        // the robot's ground speed, and the encoder reads what the robot's own wheel would.
        // biome-ignore lint/suspicious/noExplicitAny: the Jolt wheel is private to WheelDriver
        const modelRadius: number = (m.wheel as any)._wheel.GetSettings().mRadius
        const radiusRatio = modelRadius / opts.robotWheelRadius
        m.wheel.maxVelocity = opts.wheelMaxRadPerSec / radiusRatio
        brain.addSimFlow({ supplier: SimCANMotor.genSupplier(driveDev), receiver: m.wheel })

        // A hinge takes a velocity fraction typed as "Angle"; relabel the motor output to match.
        const hinge = m.hinge
        hinge.setContinuousRotation()
        if (opts.steerMaxRadPerSec) hinge.maxVelocity = opts.steerMaxRadPerSec
        const cal = cals[i]
        const hingeSign = opts.steerEncSign ?? cal.enc
        const steerCmd = opts.steerCmdSign ?? cal.cmd
        brain.addSimFlow({
            supplier: {
                supplierType: hinge.receiverType,
                getSupplierValue: () => [
                    {
                        value: steerCmd * (SimCANMotor.getPercentOutput(turnDev) ?? 0),
                        baseType: hinge.receiverType[0],
                    },
                ],
            },
            receiver: hinge,
        })

        const wheelStim = stimulusFor(m.wheel.id.guid)
        const hingeStim = stimulusFor(m.hinge.id.guid)
        if (!wheelStim || !hingeStim) throw new Error(`module ${i}: encoder stimulus not found`)
        brain.addSimFlow({ supplier: scaled(wheelStim, radiusRatio), receiver: SimCANEncoder.genReceiver(driveDev) })
        brain.addSimFlow({ supplier: scaled(hingeStim, hingeSign), receiver: SimCANEncoder.genReceiver(turnDev) })
        report.push({ module: i, hingeSign, steerCmd, calibrated: cal.ok, radiusRatio: +radiusRatio.toFixed(3) })
    })

    // Mechanism hinges: motor output -> hinge velocity fraction, hinge angle -> encoder (radians at
    // the joint; the robot code's IO turns that into its own angle convention).
    for (const mech of opts.mechanisms ?? []) {
        const h = layer.drivers.find(d => d instanceof HingeDriver && jointName(d) === mech.joint) as
            | HingeDriver
            | undefined
        if (!h) {
            report.push({ mechanism: mech.joint, missing: true })
            continue
        }
        h.maxVelocity = mech.maxRadPerSec
        const bodies = [h.constraint.GetBody1(), h.constraint.GetBody2()]
        const light = bodies[bodies[0].GetMotionProperties().GetInverseMass() > bodies[1].GetMotionProperties().GetInverseMass() ? 0 : 1]
        const height = () => light.GetCenterOfMassPosition().GetY() - h.worldAnchor.GetY()
        const want = mech.positive === "down" ? () => -height() : height
        const mcal = await calibrateHinge(h, want, mech.maxRadPerSec)
        const cmd = mcal.cmd
        const enc = mcal.enc
        brain.addSimFlow({
            supplier: {
                supplierType: h.receiverType,
                getSupplierValue: () => [
                    { value: cmd * (SimCANMotor.getPercentOutput(mech.motor) ?? 0), baseType: h.receiverType[0] },
                ],
            },
            receiver: h,
        })
        const stim = stimulusFor(h.id.guid)
        if (stim) brain.addSimFlow({ supplier: scaled(stim as Stimulus, enc), receiver: SimCANEncoder.genReceiver(mech.motor) })
        report.push({ mechanism: mech.joint, motor: mech.motor, encoder: !!stim, cmd, enc, calibrated: mcal.ok })
    }

    // Gyro: a stimulus on the chassis body, registered with the layer so it is stepped each tick.
    const rootBodyId = robot.mechanism.nodeToBody.get(robot.mechanism.rootBody)!
    const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    const gyroGuid = `SPIKE_GYRO_${opts.gyro}`
    const gyro = new GyroStimulus({ type: StimulusType.STIM_GYRO, guid: gyroGuid }, rootBodyId, identity, opts.gyro, {
        GUID: gyroGuid,
        name: "Chassis gyro",
    })
    // biome-ignore lint/suspicious/noExplicitAny: spike reaches into the layer's private map
    ;(layer as any)._stimuli.set(JSON.stringify(gyro.id), gyro)
    brain.addSimFlow({ supplier: gyro, receiver: SimGyro.genReceiver(opts.gyro) })

    return report
}

/** Everything a test needs to see what the robot is doing, in one call. */
export function swerveSnapshot(robot: MirabufSceneObject) {
    const body = World.physicsSystem.getBody(robot.getRootNodeId()!)!
    const p = body.GetCenterOfMassPosition()
    const v = body.GetLinearVelocity()
    const r = body.GetRotation()
    const yaw = Math.atan2(2 * (r.GetW() * r.GetY() + r.GetX() * r.GetZ()), 1 - 2 * (r.GetY() ** 2 + r.GetZ() ** 2))
    return {
        pos: [p.GetX(), p.GetY(), p.GetZ()].map(n => +n.toFixed(3)),
        vel: [v.GetX(), v.GetY(), v.GetZ()].map(n => +n.toFixed(3)),
        yawDeg: +((yaw * 180) / Math.PI).toFixed(1),
    }
}

const FIELD_2026 = { path: "/Downloadables/mira/fields/FRC Field 2026 v4.mira", hash: "bd223db6" }
const SWERVE_SIMPLE = {
    path: "/Downloadables/mira/private/SwerveSimple v2.mira",
    hash: "7567ce712e23bee5e3c64154d2667b19e74f98e8",
}

const sleep = (ms: number) => new Promise(r => setTimeout(r, ms))

/** Sphinx (FRC 8567) built by tools/synthesis/urdf/build_sphinx_urdf.py, served from public/. */
export const SPHINX_URDF_ZIP = "/sphinx_urdf.zip"

async function spawnURDF(url: string, name: string) {
    const { loadURDF } = await import("@/urdf/URDFLoader")
    const { default: MirabufCachingService, MiraType } = await import("@/mirabuf/MirabufLoader")
    const { spawnCachedMira } = await import("@/ui/modals/mirabuf/LibrarySpawnActions")
    const { ProgressHandle } = await import("@/components/ProgressNotificationData")
    const buffer = await (await fetch(url)).arrayBuffer()
    const { assembly } = await loadURDF(buffer, `${name}.zip`, new ProgressHandle(name))
    assembly.info!.name = name
    const info = await MirabufCachingService.storeAssemblyInCache(assembly, { miraType: MiraType.ROBOT })
    if (!info) throw new Error("could not cache URDF assembly")
    await spawnCachedMira(info)
}

/** Starts spawns without awaiting them; poll {@link setupStatus} until ready. */
export async function startSetup(spawnField: boolean, robot: "sphinx" | "swerveSimple" = "sphinx") {
    const { spawnRemote } = await import("@/ui/modals/mirabuf/LibrarySpawnActions")
    const { MiraType } = await import("@/mirabuf/MirabufLoader")
    const objs = World.sceneRenderer.mirabufSceneObjects
    const mine = () => objs.getRobots().filter(r => !r.multiplayerOwnerName)
    ;(async () => {
        if (spawnField && !objs.getField()) {
            spawnRemote({ remotePath: FIELD_2026.path, hash: FIELD_2026.hash, miraType: MiraType.FIELD, name: "FRC Field 2026" })
            for (let i = 0; i < 90 && !objs.getField(); i++) await sleep(1000)
        }
        if (mine().length === 0) {
            if (robot === "sphinx") spawnURDF(SPHINX_URDF_ZIP, "Sphinx 8567").catch(e => console.error(e))
            else spawnRemote({ remotePath: SWERVE_SIMPLE.path, hash: SWERVE_SIMPLE.hash, miraType: MiraType.ROBOT, name: "SwerveSimple v2" })
            for (let i = 0; i < 60 && mine().length === 0; i++) await sleep(1000)
        }
    })()
}

export function myRobot(): MirabufSceneObject | undefined {
    return World.sceneRenderer.mirabufSceneObjects.getRobots().find(r => !r.multiplayerOwnerName)
}

export function setupStatus() {
    const objs = World.sceneRenderer.mirabufSceneObjects
    return {
        room: World.multiplayerSystem?.roomId,
        players: World.multiplayerSystem ? World.multiplayerSystem.clientToInfoMap.size : 0,
        field: !!objs.getField(),
        robots: objs.getRobots().map(r => `${r.multiplayerOwnerName ?? "me"}:${r.brain?.brainType ?? "-"}`),
    }
}

/**
 * Xbox controller 0 as the robot code sees it. Axes in WPILib order: LX, LY, LT, RT, RX, RY.
 * WPILib's Y axes are down-positive, so "forward on the left stick" is ly = -1.
 */
export async function setJoystick(axes: { lx?: number; ly?: number; rx?: number }, port = 0) {
    const { worker, SimType } = await import("@/systems/simulation/wpilib_brain/WPILibTypes")
    worker.getValue().postMessage({
        command: "update",
        data: {
            type: "Joystick",
            device: String(port),
            data: {
                ">axes": [axes.lx ?? 0, axes.ly ?? 0, 0, 0, axes.rx ?? 0, 0],
                ">buttons": Array(10).fill(false),
                ">povs": [-1],
            },
        },
    })
    void SimType
}

export async function setRobotMode(mode: "disabled" | "teleop" | "auto", station = "blue1") {
    const { RobotSimMode } = await import("@/systems/simulation/wpilib_brain/WPILibTypes")
    const DS = (await import("@/systems/simulation/wpilib_brain/sim/SimDriverStation")).default
    DS.setStation(station as never)
    DS.setMode(mode === "teleop" ? RobotSimMode.TELEOP : mode === "auto" ? RobotSimMode.AUTO : RobotSimMode.DISABLED)
}

/** Whole player setup in one call: join is done by ?autojoin; this spawns, closes panels, attaches. */
export async function setupPlayer(
    spawnField: boolean,
    pos: [number, number, number],
    robot: "sphinx" | "swerveSimple" = "sphinx"
) {
    await startSetup(spawnField, robot)
    for (let i = 0; i < 90; i++) {
        const s = setupStatus()
        if (s.field && myRobot()) break
        await sleep(1000)
    }
    await sleep(1500)
    for (const name of ["Skip", "Finish"]) {
        ;[...document.querySelectorAll("button")].find(b => b.textContent?.trim() === name)?.click()
        await sleep(500)
    }
    const r = myRobot()!
    // biome-ignore lint/suspicious/noExplicitAny: spike teleports through a private method
    ;(r as any).setObjectPosition({ pos, yaw: 0 })
    await sleep(1500)
    const report = await attachSwerveCodeSim(r, robot === "sphinx" ? DEFAULT_8567 : { ...DEFAULT_8567, ...SWERVE_SIMPLE_SIGNS, mechanisms: [] })
    return { paused: World.physicsSystem.isPaused, status: setupStatus(), report: report.filter(o => o.hingeSign !== undefined || o.mechanism) }
}

/**
 * Forwards this browser's controllers to the robot code as WPILib Xbox controllers.
 *
 * Synthesis does not do this for WPILib robots: its SimGamepadInput is never instantiated and
 * speaks the FTC gamepad format. First connected controller -> port 0 (driver), second -> port 1
 * (operator). With no controller on port 0 the keyboard drives: WASD translate, arrows turn.
 *
 * Also re-sends `>new_data` each tick. WPILib only applies driver-station and joystick changes on
 * that signal, so without it Synthesis's own enable/disable buttons never reach the robot.
 */
let forwarder: ReturnType<typeof setInterval> | undefined
const keysDown = new Set<string>()

export async function startControllerForwarding(periodMs = 20) {
    const { worker } = await import("@/systems/simulation/wpilib_brain/WPILibTypes")
    const send = (data: unknown) => worker.getValue().postMessage({ command: "update", data })
    if (forwarder) clearInterval(forwarder)
    window.addEventListener("keydown", e => keysDown.add(e.code))
    window.addEventListener("keyup", e => keysDown.delete(e.code))
    window.addEventListener("blur", () => keysDown.clear())

    forwarder = setInterval(() => {
        const pads = [...(navigator.getGamepads?.() ?? [])].filter((p): p is Gamepad => !!p && p.connected)
        for (let port = 0; port < 2; port++) {
            const pad = pads[port]
            const data = pad ? xboxFromGamepad(pad) : port === 0 ? xboxFromKeyboard() : undefined
            if (data) send({ type: "Joystick", device: String(port), data })
        }
        send({ type: "DriverStation", device: "", data: { ">new_data": true } })
    }, periodMs)
}

export function stopControllerForwarding() {
    if (forwarder) clearInterval(forwarder)
    forwarder = undefined
}

// Browser "standard" gamepad layout -> WPILib XboxController layout.
// WPILib axes: LX, LY, LT, RT, RX, RY (triggers 0..1). Buttons: A B X Y LB RB Back Start LS RS.
function xboxFromGamepad(p: Gamepad) {
    const b = (i: number) => !!p.buttons[i]?.pressed
    const v = (i: number) => p.buttons[i]?.value ?? 0
    return {
        ">axes": [p.axes[0] ?? 0, p.axes[1] ?? 0, v(6), v(7), p.axes[2] ?? 0, p.axes[3] ?? 0],
        ">buttons": [b(0), b(1), b(2), b(3), b(4), b(5), b(8), b(9), b(10), b(11)],
        ">povs": [pov(b(12), b(15), b(13), b(14))],
    }
}

function xboxFromKeyboard() {
    const k = (c: string) => (keysDown.has(c) ? 1 : 0)
    return {
        ">axes": [k("KeyD") - k("KeyA"), k("KeyS") - k("KeyW"), 0, 0, k("ArrowRight") - k("ArrowLeft"), 0],
        ">buttons": Array(10).fill(false),
        ">povs": [-1],
    }
}

function pov(up: boolean, right: boolean, down: boolean, left: boolean): number {
    const x = (right ? 1 : 0) - (left ? 1 : 0)
    const y = (up ? 1 : 0) - (down ? 1 : 0)
    if (x === 0 && y === 0) return -1
    return (Math.round((Math.atan2(x, y) * 180) / Math.PI / 45) * 45 + 360) % 360
}
