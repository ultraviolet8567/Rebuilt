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
    hingeSign?: number // -1 if a model's steering reads clockwise-positive
}

export const DEFAULT_8567: SwerveCodeSimOptions = {
    driveIds: [10, 11, 12, 13],
    turnIds: [20, 21, 22, 23],
    gyro: "Pigeon2[30]",
    wheelMaxRadPerSec: 102.8,
    robotWheelRadius: 0.04863,
}

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

export function attachSwerveCodeSim(robot: MirabufSceneObject, opts: SwerveCodeSimOptions = DEFAULT_8567) {
    const layer = World.simulationSystem.getSimulationLayer(robot.mechanism)!
    if (!(robot.brain instanceof WPILibBrain)) robot.brain = new WPILibBrain(robot, "wpilib")
    const brain = robot.brain as WPILibBrain

    const wheels = layer.drivers.filter((d): d is WheelDriver => d instanceof WheelDriver)
    const hinges = layer.drivers.filter((d): d is HingeDriver => d instanceof HingeDriver)
    if (wheels.length !== 4 || hinges.length !== 4) throw new Error(`need 4 wheels + 4 hinges`)

    const report: Record<string, unknown>[] = []
    const byModule: { wheel?: WheelDriver; hinge?: HingeDriver }[] = [{}, {}, {}, {}]
    wheels.forEach(w => {
        const local = toChassis(robot, wheelWorldPos(w))
        const i = moduleIndex(local)
        byModule[i].wheel = w
        report.push({ module: i, wheel: w.idStr, local })
    })
    hinges.forEach(h => {
        const a = h.worldAnchor
        const local = toChassis(robot, { x: a.GetX(), y: a.GetY(), z: a.GetZ() })
        const i = moduleIndex(local)
        byModule[i].hinge = h
        report.push({ module: i, hinge: h.displayName(), local })
    })

    const stimulusFor = (guid: string) =>
        layer.stimuli.find(s => s.id.guid === guid && s.id.type === StimulusType.STIM_ENCODER)

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
        // Measured on SwerveSimple (all hinge axes point down): Synthesis's hinge angle already
        // reads counter-clockwise-from-above positive, as WPILib expects. Flipping it made the
        // robot unable to rotate in place. Kept as a knob for models that disagree.
        const hinge = m.hinge
        hinge.setContinuousRotation()
        const hingeSign = opts.hingeSign ?? 1
        brain.addSimFlow({
            supplier: {
                supplierType: hinge.receiverType,
                getSupplierValue: () => [
                    { value: hingeSign * (SimCANMotor.getPercentOutput(turnDev) ?? 0), baseType: hinge.receiverType[0] },
                ],
            },
            receiver: hinge,
        })

        const wheelStim = stimulusFor(m.wheel.id.guid)
        const hingeStim = stimulusFor(m.hinge.id.guid)
        if (!wheelStim || !hingeStim) throw new Error(`module ${i}: encoder stimulus not found`)
        const scaled = (stim: Stimulus, k: number) => ({
            supplierType: stim.supplierType,
            getSupplierValue: () =>
                // biome-ignore lint/suspicious/noExplicitAny: [position, velocity] pair
                (stim.getSupplierValue() as any[]).map(v => ({ ...v, value: k * v.value })),
        })
        brain.addSimFlow({ supplier: scaled(wheelStim, radiusRatio), receiver: SimCANEncoder.genReceiver(driveDev) })
        brain.addSimFlow({ supplier: scaled(hingeStim, hingeSign), receiver: SimCANEncoder.genReceiver(turnDev) })
        report.push({ module: i, hingeSign, radiusRatio: +radiusRatio.toFixed(3) })
    })

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

/** Starts spawns without awaiting them; poll {@link setupStatus} until ready. */
export async function startSetup(spawnField: boolean) {
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
            spawnRemote({ remotePath: SWERVE_SIMPLE.path, hash: SWERVE_SIMPLE.hash, miraType: MiraType.ROBOT, name: "SwerveSimple v2" })
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
export async function setupPlayer(spawnField: boolean, pos: [number, number, number]) {
    await startSetup(spawnField)
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
    const report = attachSwerveCodeSim(r)
    return { paused: World.physicsSystem.isPaused, status: setupStatus(), report: report.filter(o => o.hingeSign !== undefined) }
}
