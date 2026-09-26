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
import * as THREE from "three"
import type Jolt from "@synthesis.adsk/jolt-physics"
import JOLT from "@/util/loading/JoltSyncLoader"
import { SimInput } from "@/systems/simulation/wpilib_brain/SimInput"
import SimGeneric from "@/systems/simulation/wpilib_brain/sim/SimGeneric"
import { SimType } from "@/systems/simulation/wpilib_brain/WPILibTypes"
import World from "@/systems/World"
import type MirabufSceneObject from "@/mirabuf/MirabufSceneObject"
import WPILibBrain from "@/systems/simulation/wpilib_brain/WPILibBrain"
import WheelDriver from "@/systems/simulation/driver/WheelDriver"
import HingeDriver from "@/systems/simulation/driver/HingeDriver"
import IntakeDriver from "@/systems/simulation/driver/IntakeDriver"
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
    mechanisms?: { joint: string; motor: string; maxRadPerSec: number; positive: "down" | "up" | "forward" }[]
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
        // Deploying swings the arm out FORWARD. "down" is ambiguous from stowed (arm pointing up):
        // both ways lower it, and calibration once folded the arm back into the robot.
        { joint: "dof_intake_pivot", motor: "Pivot[5]", maxRadPerSec: 5.28, positive: "forward" }, // NEO / (45 * 40/16)
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
        const chassis = World.physicsSystem.getBody(robot.getRootNodeId()!)!
        const forward = () => {
            const c = light.GetCenterOfMassPosition()
            const [cx, cz] = [c.GetX(), c.GetZ()]
            const a = h.worldAnchor
            const [ax, az] = [a.GetX(), a.GetZ()]
            const q = chassis.GetRotation()
            const [x, y, z, w] = [q.GetX(), q.GetY(), q.GetZ(), q.GetW()]
            const fx = 1 - 2 * (y * y + z * z)
            const fz = 2 * (x * z - w * y) // chassis +X (forward) in world
            return (cx - ax) * fx + (cz - az) * fz
        }
        const want = mech.positive === "down" ? () => -height() : mech.positive === "forward" ? forward : height
        const mcal = await calibrateHinge(h, want, mech.maxRadPerSec)
        const cmd = mcal.cmd
        const enc = mcal.enc
        // Return to the model's zero (stowed intake, lowered hood): the robot code boots believing
        // it is there, and the pickup / launch points are computed for that pose.
        const up = Math.sign(mcal.trials.find(t => Math.abs(t.dq) > 0.02)?.dq ?? 1) * Math.sign(mcal.trials.find(t => Math.abs(t.dq) > 0.02)?.u ?? 1)
        for (let i = 0; i < 40 && Math.abs(h.constraint.GetCurrentAngle()) > 0.01; i++) {
            const qNow = h.constraint.GetCurrentAngle()
            h.accelerationDirection = -up * Math.max(-0.5, Math.min(0.5, 3 * qNow))
            await sleep(50)
        }
        h.accelerationDirection = 0
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
/** Multiplies every shot's exit speed; for calibrating the efficiency table (1 = as measured). */
export const shotTuning = { gain: 1 }

export const SPHINX_URDF_ZIP = "/sphinx_urdf.zip"

async function spawnURDF(url: string, name: string) {
    const { loadURDF } = await import("@/urdf/URDFLoader")
    const { default: MirabufCachingService, MiraType } = await import("@/mirabuf/MirabufLoader")
    const { spawnCachedMira } = await import("@/ui/modals/mirabuf/LibrarySpawnActions")
    const { ProgressHandle } = await import("@/components/ProgressNotificationData")
    const buffer = await (await fetch(url)).arrayBuffer()
    await loadMeta(url)
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
export type Station = `${"red" | "blue"}${1 | 2 | 3}`

/**
 * Whole player setup in one call (joining is done by ?autojoin): spawn, close the setup panels,
 * attach the robot code, and put the robot at `where` -- an alliance station ("red2") or a raw
 * Synthesis position.
 */
export async function setupPlayer(
    spawnField: boolean,
    where: [number, number, number] | Station,
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
    const station = typeof where === "string" ? where : undefined
    if (station) {
        r.alliance = station.startsWith("red") ? "red" : "blue"
        r.station = Number(station.slice(-1)) as 1 | 2 | 3
        r.moveToSpawnLocation()
    } else {
        // biome-ignore lint/suspicious/noExplicitAny: spike teleports through a private method
        ;(r as any).setObjectPosition({ pos: where, yaw: 0 })
    }
    await sleep(1500)
    const report = await attachSwerveCodeSim(r, robot === "sphinx" ? DEFAULT_8567 : { ...DEFAULT_8567, ...SWERVE_SIMPLE_SIGNS, mechanisms: [] })
    const extras = robot === "sphinx" ? attachSphinxExtras(r) : {}
    if (station) {
        placeAtStation(r, r.alliance as "red" | "blue", r.station as 1 | 2 | 3) // back to the start after calibration
        myStation = station
    }
    return {
        paused: World.physicsSystem.isPaused,
        status: setupStatus(),
        extras,
        report: report.filter(o => o.hingeSign !== undefined || o.mechanism),
    }
}

let myStation: Station = "blue1"

/**
 * Follow Synthesis match mode with the robot program's driver station: disabled before the
 * match, autonomous for the auto period (the robot runs its selected auto), teleop after, and
 * disabled when the match ends. Outside a match (sandbox) the robot stays in teleop so people can
 * practise.
 */
export async function followMatchMode() {
    const { MatchModeType } = await import("@/systems/match_mode/MatchModeTypes")
    const EventSystem = (await import("@/systems/EventSystem")).default
    // biome-ignore lint/suspicious/noExplicitAny: event payload shape differs between versions
    EventSystem.listen("MatchStateChangedEvent", (e: any) => {
        const mode = e?.data?.mode ?? e?.mode
        const next =
            mode === MatchModeType.AUTONOMOUS
                ? "auto"
                : mode === MatchModeType.TELEOP || mode === MatchModeType.ENDGAME || mode === MatchModeType.SANDBOX
                  ? "teleop"
                  : "disabled"
        setRobotMode(next as "auto" | "teleop" | "disabled", myStation)
    })
    await setRobotMode("teleop", myStation)
}

/**
 * One-step start for players: ?autojoin=ROOM&sphinx=red2[&field=1][&codesim=ws://...].
 * `field=1` only for the host, who brings the field; everyone else receives it.
 */
export async function autoStart(params: URLSearchParams) {
    const station = (params.get("sphinx") ?? "blue1") as Station
    const { globalAddToast } = await import("@/components/GlobalUIControls")
    globalAddToast("info", "8567 simulation", `Setting up ${station}. Keep this tab visible.`)
    try {
        // Joining happens in the background from ?autojoin; give it a moment, then refuse to
        // carry on alone -- a player who silently misses the room sees nobody else.
        const room = params.get("autojoin")
        for (let i = 0; i < 60 && room && !World.multiplayerSystem?.roomId; i++) await sleep(500)
        if (room && World.multiplayerSystem?.roomId !== room) {
            throw new Error(`could not join room ${room}. Check the link, or ask the host whether the relay is running.`)
        }
        const r = await setupPlayer(params.get("field") === "1", station, "sphinx")
        await startControllerForwarding()
        await followMatchMode()
        const view = params.get("view") as View
        await setView(VIEWS.includes(view) ? view : "driver", station)
        installViewKey()
        const ok = r.report.every((x: Record<string, unknown>) => x.calibrated !== false)
        globalAddToast(ok ? "info" : "warning", "8567 simulation", ok ? `Ready at ${station}. Drive!` : "Ready, but a joint did not calibrate: keep the tab visible and reload.")
        // biome-ignore lint/suspicious/noExplicitAny: status flag for scripted players and tests
        ;(window as any).__sphinx = { ready: true, ...r }
        // biome-ignore lint/suspicious/noExplicitAny: handles for scripted players and tests
        ;(window as any).__W = World
        return r
    } catch (e) {
        globalAddToast("error", "8567 simulation", `Setup failed: ${e}`)
        // biome-ignore lint/suspicious/noExplicitAny: status flag for scripted players and tests
        ;(window as any).__sphinx = { ready: false, error: String(e) }
        throw e
    }
}

/**
 * Camera views, cycled with V:
 * - "driver": standing behind your own driver station, turning to keep your robot in view -- what
 *   a real driver sees. The field's camera points supply the stations ("Red Alliance 2", ...);
 *   station 1 is on the drivers' left, 3 on their right.
 * - "station": the same spot, looking at the field centre without turning.
 * - "follow": above and behind your robot, from your own alliance's side.
 */
export const VIEWS = ["driver", "station", "follow"] as const
export type View = (typeof VIEWS)[number]
let currentView: View = "driver"

export async function setView(view: View, station: Station = myStation) {
    const { CustomFieldViewControls, CustomTargetControls } = await import("@/systems/scene/CameraControls")
    const field = World.sceneRenderer.mirabufSceneObjects.getField()
    const robot = myRobot()
    const alliance = station.startsWith("red") ? "Red" : "Blue"
    const name = `${alliance} Alliance ${station.slice(-1)}`
    const index = field?.fieldPreferences?.cameraPoints?.findIndex(p => p.name === name) ?? -1
    if (view !== "follow" && field && index >= 0) {
        World.sceneRenderer.setCameraControls("FieldView")
        const c = World.sceneRenderer.currentCameraControls
        if (c instanceof CustomFieldViewControls) {
            c.selectPoint(field, index)
            c.focusRobot(view === "driver" ? robot : undefined)
        }
    } else {
        // "follow", or a field without driver-station camera points.
        World.sceneRenderer.setCameraControls("Target")
        const c = World.sceneRenderer.currentCameraControls
        if (c instanceof CustomTargetControls && robot) {
            c.focusProvider = robot
            // The focus change re-syncs the orbit to wherever the camera was on the next frame;
            // then sit above and behind the robot on your own alliance's side, facing the other
            // alliance, so "up" on the stick is still "up" on screen. Blue's wall is at +X.
            await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))
            c.setImmediateCoordinates({ theta: station.startsWith("blue") ? Math.PI / 2 : -Math.PI / 2, phi: -Math.PI / 6, r: 4 })
        }
        view = "follow"
    }
    currentView = view
    return view
}

let viewKeysInstalled = false
function installViewKey() {
    if (viewKeysInstalled) return
    viewKeysInstalled = true
    window.addEventListener("keydown", async e => {
        if (e.code !== "KeyV" || e.repeat) return
        if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return
        const next = VIEWS[(VIEWS.indexOf(currentView) + 1) % VIEWS.length]
        const shown = await setView(next)
        const { globalAddToast } = await import("@/components/GlobalUIControls")
        const label = { driver: "Driver station, turning to your robot", station: "Driver station, fixed", follow: "Chase camera above your robot" }
        globalAddToast("info", "Camera", `${label[shown]} (V to change)`)
    })
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
let intakeLatched = false // C toggles the intake on and off, so it can run while you drive

export async function startControllerForwarding(periodMs = 20) {
    const { worker } = await import("@/systems/simulation/wpilib_brain/WPILibTypes")
    const send = (data: unknown) => worker.getValue().postMessage({ command: "update", data })
    if (forwarder) clearInterval(forwarder)
    window.addEventListener("keydown", e => {
        keysDown.add(e.code)
        if (e.code === "KeyC" && !e.repeat) {
            intakeLatched = !intakeLatched
            import("@/components/GlobalUIControls").then(m => m.globalAddToast("info", "Intake", intakeLatched ? "On (C to stop)" : "Off"))
        }
    })
    window.addEventListener("keyup", e => keysDown.delete(e.code))
    window.addEventListener("blur", () => keysDown.clear())

    forwarder = setInterval(() => {
        const pads = [...(navigator.getGamepads?.() ?? [])].filter((p): p is Gamepad => !!p && p.connected)
        // One controller: it is both driver and operator (the robot code binds them to ports 0
        // and 1). The overlaps are usable: RT aims and shoots at once. No controller: keyboard.
        const driver = pads[0] ? xboxFromGamepad(pads[0]) : xboxFromKeyboard("driver")
        const operator = pads[1] ? xboxFromGamepad(pads[1]) : pads[0] ? driver : xboxFromKeyboard("operator")
        send({ type: "Joystick", device: "0", data: driver })
        send({ type: "Joystick", device: "1", data: operator })
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

/**
 * Keyboard play. Driver: WASD move, arrow keys turn, Shift slow, Space aim at hub.
 * Operator: E deploy intake + run it (C toggles that on/off), Q stow, F shoot (with aim),
 * R reverse the funnel.
 */
function xboxFromKeyboard(role: "driver" | "operator") {
    const k = (c: string) => (keysDown.has(c) ? 1 : 0)
    const b = (c: string) => keysDown.has(c)
    const buttons = Array(10).fill(false)
    const axes = [0, 0, 0, 0, 0, 0]
    if (role === "driver") {
        axes[0] = k("KeyD") - k("KeyA")
        axes[1] = k("KeyS") - k("KeyW")
        axes[4] = k("ArrowRight") - k("ArrowLeft")
        axes[3] = b("Space") || b("KeyF") ? 1 : 0 // RT: aim at hub (also while shooting)
        buttons[5] = b("ShiftLeft") || b("ShiftRight") // RB: slow
    } else {
        buttons[0] = b("KeyE") || intakeLatched // A: deploy intake
        buttons[4] = b("KeyE") || intakeLatched // LB: funnel in
        if (b("KeyQ")) intakeLatched = false
        buttons[3] = b("KeyQ") // Y: stow
        buttons[1] = b("KeyR") // B: funnel out
        axes[3] = b("KeyF") ? 1 : 0 // RT: ranged shot
    }
    return { ">axes": axes, ">buttons": buttons, ">povs": [-1] }
}

function pov(up: boolean, right: boolean, down: boolean, left: boolean): number {
    const x = (right ? 1 : 0) - (left ? 1 : 0)
    const y = (up ? 1 : 0) - (down ? 1 : 0)
    if (x === 0 && y === 0) return -1
    return (Math.round((Math.atan2(x, y) * 180) / Math.PI / 45) * 45 + 360) % 360
}

/* ------------------------------------------------------------------------------------------------
 * Game pieces and field position
 * ---------------------------------------------------------------------------------------------- */

export type SphinxMeta = {
    hopper?: { min: number[]; max: number[] } // robot (URDF) frame, metres
    intake: { link: string; point: number[]; diameter: number; maxPieces: number }
    launcher: {
        link: string
        point: number[]
        direction: number[]
        flywheelRadius: number
        efficiencyByRpm: [number, number][] // piecewise-linear, clamped at the ends
    }
}
let sphinxMeta: SphinxMeta | undefined

async function loadMeta(url: string) {
    const JSZip = (await import("jszip")).default
    const zip = await JSZip.loadAsync(await (await fetch(url)).arrayBuffer())
    const f = Object.values(zip.files).find(x => x.name.endsWith("sim.json"))
    sphinxMeta = f ? JSON.parse(await f.async("text")) : undefined
}

// URDF (x forward, y left, z up) -> Synthesis Y-up, the same conversion the URDF importer uses.
const yup = (v: number[]) => new THREE.Vector3(v[0], v[2], -v[1])

function nodeForLink(robot: MirabufSceneObject, link: string): string | undefined {
    // biome-ignore lint/suspicious/noExplicitAny: parser rigid nodes carry the URDF link names as parts
    const nodes: any[] = [...(robot.mirabufInstance.parser.rigidNodes as any).values()]
    return nodes.find(n => [...(n.parts ?? [])].includes(link))?.id
}

function bodyMatrix(id: Jolt.BodyID) {
    const b = World.physicsSystem.getBody(id)!
    const t = b.GetPosition()
    const q = b.GetRotation()
    const pos = new THREE.Vector3(t.GetX(), t.GetY(), t.GetZ()) // copy at once: Jolt reuses these
    const rot = new THREE.Quaternion(q.GetX(), q.GetY(), q.GetZ(), q.GetW())
    return new THREE.Matrix4().compose(pos, rot, new THREE.Vector3(1, 1, 1))
}

function hingeAnchor(robot: MirabufSceneObject, joint: string) {
    const layer = World.simulationSystem.getSimulationLayer(robot.mechanism)!
    const h = layer.drivers.find(d => jointName(d) === joint) as HingeDriver | undefined
    if (!h) return undefined
    const a = h.worldAnchor
    return new THREE.Vector3(a.GetX(), a.GetY(), a.GetZ())
}

/**
 * Pose of a point on a link in the frame of the physics body that carries the link, as the
 * column-major array Synthesis stores in intake / ejector preferences. `dir` becomes the frame's
 * +Z, which is the direction Synthesis launches a held piece.
 *
 * Measured: every link body's frame coincides with the chassis frame when its joint is at zero
 * (the URDF importer builds all bodies at the robot origin), so the answer depends only on the
 * chassis frame and the joint's anchor, which is fixed to the chassis. It does not matter where
 * the joint happens to be when this runs; using the link body's current pose did, and put the
 * pickup zone inside the robot when calibration left the arm off zero.
 */
function linkPointToBody(robot: MirabufSceneObject, link: string, joint: string, point: number[], dir?: number[]) {
    const node = nodeForLink(robot, link)!
    const chassis = bodyMatrix(robot.mechanism.nodeToBody.get(robot.rootNodeId)!)
    const toChassis = chassis.clone().invert()
    const anchor = hingeAnchor(robot, joint)!.applyMatrix4(toChassis) // joint origin, chassis frame
    const pos = anchor.add(yup(point))
    const z = (dir ? yup(dir) : new THREE.Vector3(0, 0, 1)).normalize()
    const x = new THREE.Vector3(0, 1, 0).cross(z)
    if (x.lengthSq() < 1e-6) x.set(1, 0, 0)
    x.normalize()
    const y = z.clone().cross(x)
    const local = new THREE.Matrix4().makeBasis(x, y, z).setPosition(pos)
    return { node, delta: local.toArray() }
}

export function attachGamePieces(robot: MirabufSceneObject, meta: SphinxMeta) {
    const intake = linkPointToBody(robot, meta.intake.link, "dof_intake_pivot", meta.intake.point)
    const launcher = linkPointToBody(robot, meta.launcher.link, "dof_hood", meta.launcher.point, meta.launcher.direction)
    robot.intakePreferences = {
        ...robot.intakePreferences,
        deltaTransformation: intake.delta,
        zoneDiameter: meta.intake.diameter,
        parentNode: intake.node,
        showZoneAlways: false,
        maxPieces: meta.intake.maxPieces,
    }
    robot.ejectorPreferences = {
        ...robot.ejectorPreferences,
        deltaTransformation: launcher.delta,
        ejectorVelocity: 8,
        parentNode: launcher.node,
        ejectOrder: "FIFO",
    }
    robot.updateIntakeSensor()
    return { intakeNode: intake.node, launcherNode: launcher.node }
}

function interpolate(table: [number, number][], x: number) {
    if (x <= table[0][0]) return table[0][1]
    for (let i = 1; i < table.length; i++) {
        const [x0, y0] = table[i - 1]
        const [x1, y1] = table[i]
        if (x <= x1) return y0 + ((y1 - y0) * (x - x0)) / (x1 - x0)
    }
    return table[table.length - 1][1]
}

/** Reads the robot code's roller outputs each physics step and runs Synthesis's intake / ejector. */
/** Fuel diameter (5.91 in). */
const FUEL_D = 0.15

/**
 * Where held fuel sits. Synthesis parks every held piece at the ejector, so 40 balls would sit
 * inside one another on top of the shooter and the hopper would look empty. Here each held ball
 * gets its own spot in the hopper: a grid between the side plates, bottom layer first, nearest
 * the shooter first. The ball due to fire next moves up to the ejector just before it goes, and
 * balls above an emptied spot drop into it.
 */
class Hopper {
    readonly slots: THREE.Vector3[] = [] // chassis-body frame
    private readonly _below: number[] = [] // index of the slot directly underneath, or -1

    constructor(box: { min: number[]; max: number[] }) {
        const n = (a: number, b: number) => Math.max(1, Math.floor((b - a) / FUEL_D))
        const [nx, ny, nz] = [0, 1, 2].map(i => n(box.min[i], box.max[i] + FUEL_D / 2))
        const at = (i: number, k: number, count: number) =>
            box.min[i] + (box.max[i] - box.min[i] - count * FUEL_D) / 2 + FUEL_D * (k + 0.5)
        for (let z = 0; z < nz; z++)
            for (let x = 0; x < nx; x++)
                for (let y = 0; y < ny; y++) {
                    this.slots.push(yup([at(0, x, nx), at(1, y, ny), box.min[2] + FUEL_D * (z + 0.5)]))
                    this._below.push(z === 0 ? -1 : this.slots.length - 1 - nx * ny)
                }
    }

    below(slot: number) {
        return this._below[slot]
    }
}

class GamePieceControl extends SimInput {
    private _lastShot = 0
    private _headSince = 0
    private _hopper?: Hopper
    constructor(
        private _robot: MirabufSceneObject,
        private _meta: SphinxMeta
    ) {
        super("GamePieceControl")
        if (_meta.hopper) this._hopper = new Hopper(_meta.hopper)
    }

    /** Re-aim a held piece at a new spot, moving there over `seconds`. */
    // biome-ignore lint/suspicious/noExplicitAny: ejectable internals are private to Synthesis
    private moveHeld(e: any, parentBody: Jolt.BodyID, delta: THREE.Matrix4, seconds: number) {
        const gp = e.gamePieceBodyId && World.physicsSystem.getBody(e.gamePieceBodyId)
        if (gp) {
            const c = gp.GetCenterOfMassPosition()
            const q = gp.GetRotation()
            e._startTranslation = new THREE.Vector3(c.GetX(), c.GetY(), c.GetZ())
            e._startRotation = new THREE.Quaternion(q.GetX(), q.GetY(), q.GetZ(), q.GetW())
            e._animationStartTime = performance.now()
            e._animationDuration = seconds
        }
        e._parentBodyId = parentBody
        e._deltaTransformation = delta
    }

    /** Lay the held pieces out in the hopper; returns true once the next shot is at the ejector. */
    // biome-ignore lint/suspicious/noExplicitAny: ejectable internals are private to Synthesis
    private arrangeHopper(held: any[], now: number): boolean {
        const hopper = this._hopper
        const prefs = this._robot.ejectorPreferences
        if (!hopper || !prefs || held.length === 0) return true
        const chassis = this._robot.mechanism.nodeToBody.get(this._robot.rootNodeId)!
        const launcher = this._robot.mechanism.nodeToBody.get(prefs.parentNode ?? this._robot.rootNodeId)!
        const exitDelta = new THREE.Matrix4().fromArray(prefs.deltaTransformation)
        const slotDelta = (i: number) => new THREE.Matrix4().setPosition(hopper.slots[i])

        // New pieces (Synthesis aimed them at the ejector) go to the lowest free spot.
        const used = new Set(held.map(e => e.__slot).filter((x: number | undefined) => x !== undefined && x >= 0))
        for (const e of held) {
            if (e.__slot !== undefined) continue
            const free = hopper.slots.findIndex((_, i) => !used.has(i))
            e.__slot = free
            used.add(free)
            if (free >= 0) {
                e._parentBodyId = chassis // keeps Synthesis's pickup animation, now ending here
                e._deltaTransformation = slotDelta(free)
            }
        }
        // Anything over an empty spot drops into it.
        for (const e of held) {
            if (e.__slot === undefined || e.__slot < 0) continue
            const b = hopper.below(e.__slot)
            if (b >= 0 && !used.has(b)) {
                used.delete(e.__slot)
                used.add(b)
                e.__slot = b
                this.moveHeld(e, chassis, slotDelta(b), 0.15)
            }
        }
        // The next shot: the lowest ball nearest the shooter, brought up to the ejector.
        if (held[0].__slot !== -2) {
            let best = 0
            held.forEach((e, i) => {
                if ((e.__slot ?? 1e9) >= 0 && (e.__slot ?? 1e9) < (held[best].__slot ?? 1e9)) best = i
            })
            const [head] = held.splice(best, 1)
            held.unshift(head)
            used.delete(head.__slot)
            head.__slot = -2
            this.moveHeld(head, launcher, exitDelta, 0.08)
            this._headSince = now
        }
        return now - this._headSince >= 80
    }
    public update(_dt: number) {
        const out = (dev: string) => SimCANMotor.getPercentOutput(dev) ?? 0
        // Funnel pulling in -> collect. The arm's position decides whether the zone reaches fuel.
        // Synthesis's own IntakeDriver copies its value onto the robot every frame, so set the
        // driver rather than the robot's flag.
        const on = out("Funnel[6]") > 0.2
        const layer = World.simulationSystem.getSimulationLayer(this._robot.mechanism)
        const intakeDriver = layer?.drivers.find(d => d instanceof IntakeDriver) as IntakeDriver | undefined
        if (intakeDriver) intakeDriver.value = on
        this._robot.intakeActive = on
        const rpm = out("Flywheel[1]") * 10000 // FlywheelIOSynthesis.kRpmScale
        const feeding = out("Kicker[3]") > 0.3
        const now = performance.now()
        // biome-ignore lint/suspicious/noExplicitAny: held pieces are private to the scene object
        const held: any[] = (this._robot as any)._ejectables
        const ready = this.arrangeHopper(held, now)
        if (feeding && rpm > 500 && held.length > 0 && ready && now - this._lastShot >= 100) {
            this._lastShot = now
            const surface = (rpm * 2 * Math.PI) / 60 * this._meta.launcher.flywheelRadius
            held[0]._ejectVelocity = surface * interpolate(this._meta.launcher.efficiencyByRpm, rpm) * shotTuning.gain
            this._robot.eject()
        }
    }
}

/**
 * Publishes the robot's pose in the robot code's field frame (blue alliance wall at x = 0) on
 * the FieldPoseXY[90] / FieldPoseTheta[91] encoder channels VisionIOSynthesis reads. Measured on
 * the 2026 field: blue is +X, field centre at the origin, hub centres at X = +-3.65 on Z = 0.
 */
class FieldPosePublisher extends SimInput {
    private _count = 0
    private _last?: THREE.Vector3
    constructor(private _robot: MirabufSceneObject) {
        super("FieldPose")
    }
    public bump() {
        this._count++
    }
    public update(_dt: number) {
        const m = bodyMatrix(this._robot.mechanism.nodeToBody.get(this._robot.rootNodeId)!)
        const pos = new THREE.Vector3().setFromMatrixPosition(m)
        const fwd = new THREE.Vector3(1, 0, 0).transformDirection(m)
        if (this._last && pos.distanceTo(this._last) > 0.3) this._count++ // teleported
        if (this._count === 0) this._count = 1
        this._last = pos
        const x = 8.27 - pos.x
        const y = 4.041 + pos.z
        const heading = Math.atan2(-fwd.z, fwd.x) + Math.PI
        SimGeneric.set(SimType.CAN_ENCODER, "FieldPoseXY[90]", ">position", x)
        SimGeneric.set(SimType.CAN_ENCODER, "FieldPoseXY[90]", ">velocity", y)
        SimGeneric.set(SimType.CAN_ENCODER, "FieldPoseTheta[91]", ">position", Math.atan2(Math.sin(heading), Math.cos(heading)))
        SimGeneric.set(SimType.CAN_ENCODER, "FieldPoseTheta[91]", ">velocity", this._count)
    }
}
let posePublisher: FieldPosePublisher | undefined

export function attachSphinxExtras(robot: MirabufSceneObject) {
    const brain = robot.brain as WPILibBrain
    // biome-ignore lint/suspicious/noExplicitAny: re-attaching must not stack inputs
    ;(brain as any)._simInputs = []
    posePublisher = new FieldPosePublisher(robot)
    brain.addSimInput(posePublisher)
    if (!sphinxMeta) return { gamePieces: false }
    const nodes = attachGamePieces(robot, sphinxMeta)
    brain.addSimInput(new GamePieceControl(robot, sphinxMeta))
    return { gamePieces: true, ...nodes }
}

/** Put this player's robot on its alliance station and tell the robot code where it is. */
export function placeAtStation(robot: MirabufSceneObject, alliance: "red" | "blue", station: 1 | 2 | 3) {
    robot.alliance = alliance
    robot.station = station
    robot.moveToSpawnLocation()
    posePublisher?.bump()
}

/** Start a Synthesis match for everyone in the room (what the Start Match button does). */
export async function startMatch(times?: { autonomousTime?: number; teleopTime?: number; endgameTime?: number }) {
    const MatchMode = (await import("@/systems/match_mode/MatchMode")).default.getInstance()
    // biome-ignore lint/suspicious/noExplicitAny: config shape comes from MatchModeConfigPanel
    const config = (MatchMode as any)._matchModeConfig
    if (times) MatchMode.setMatchModeConfig({ ...config, ...times })
    await MatchMode.start(null, true, true)
}
