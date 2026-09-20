package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.SimBattery;

/**
 * Physics model of one swerve module.
 *
 * <p>This talks to no vendor code at all. The previous simulation drove REV's {@code SparkFlexSim}
 * / {@code SparkMaxSim} shims, which meant every quirk of that library leaked into the robot code:
 * {@code iterate()} ignores the configured inversion, overwrites the encoder position with its own
 * copy (so {@code resetEncoders()} had to poke the sim object as well), and low-pass filters the
 * velocity it hands back. Each of those needed a workaround in a file that also runs on the real
 * robot.
 *
 * <p>Because the IO interface is the seam, the simulation can instead be an honest
 * motor-plus-gearbox model with its own controllers, and the real-robot class keeps no simulation
 * code whatsoever. This is the arrangement used by every corpus robot that adopted an IO layer.
 */
public class ModuleIOSim implements ModuleIO {
    private static final DCMotor kDriveGearbox = DCMotor.getNeoVortex(1);
    private static final DCMotor kTurnGearbox = DCMotor.getNEO(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final PIDController driveController =
            new PIDController(DriveConstants.kDriveKp, 0.0, DriveConstants.kDriveKd);
    private final PIDController turnController =
            new PIDController(DriveConstants.kTurnKp, 0.0, DriveConstants.kTurnKd);

    private boolean driveClosedLoop = false;
    private double driveVelocitySetpointMps = 0.0;
    private double driveVolts = 0.0;
    private double turnVolts = 0.0;

    public ModuleIOSim() {
        driveSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                kDriveGearbox,
                                DriveConstants.kDriveSimMOI,
                                DriveConstants.kDriveGearRatio),
                        kDriveGearbox);
        turnSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                kTurnGearbox,
                                DriveConstants.kTurnSimMOI,
                                DriveConstants.kTurnGearRatio),
                        kTurnGearbox);
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (driveClosedLoop) {
            // Mirror what the Spark does on the real robot: kS + kV feedforward plus a PID trim.
            double measured = wheelVelocityMetersPerSec();
            double ff =
                    Math.signum(driveVelocitySetpointMps) * DriveConstants.kDriveKs
                            + DriveConstants.kDriveKv * driveVelocitySetpointMps;
            driveVolts =
                    MathUtil.clamp(
                            ff + driveController.calculate(measured, driveVelocitySetpointMps),
                            -12.0,
                            12.0);
        }

        driveSim.setInputVoltage(driveVolts);
        turnSim.setInputVoltage(turnVolts);
        driveSim.update(Constants.kLoopPeriodSecs);
        turnSim.update(Constants.kLoopPeriodSecs);

        inputs.driveConnected = true;
        inputs.drivePositionMeters =
                driveSim.getAngularPositionRad() * DriveConstants.kWheelRadiusMeters;
        inputs.driveVelocityMetersPerSec = wheelVelocityMetersPerSec();
        inputs.driveAppliedVolts = driveVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.turnConnected = true;
        inputs.absoluteEncoderConnected = true;
        inputs.turnPosition =
                new Rotation2d(
                        MathUtil.inputModulus(turnSim.getAngularPositionRad(), -Math.PI, Math.PI));
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        inputs.absoluteEncoderVolts =
                (MathUtil.inputModulus(turnSim.getAngularPositionRad(), 0.0, 2 * Math.PI)
                                / (2 * Math.PI))
                        * 5.0;

        SimBattery.addCurrent(driveSim.getCurrentDrawAmps());
        SimBattery.addCurrent(turnSim.getCurrentDrawAmps());
    }

    private double wheelVelocityMetersPerSec() {
        return driveSim.getAngularVelocityRadPerSec() * DriveConstants.kWheelRadiusMeters;
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec) {
        driveClosedLoop = true;
        driveVelocitySetpointMps = velocityMetersPerSec;
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveClosedLoop = false;
        driveVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnVolts =
                MathUtil.clamp(
                        turnController.calculate(
                                turnSim.getAngularPositionRad(), rotation.getRadians()),
                        -12.0,
                        12.0);
    }

    @Override
    public void stop() {
        driveClosedLoop = false;
        driveVolts = 0.0;
        turnVolts = 0.0;
    }
}
