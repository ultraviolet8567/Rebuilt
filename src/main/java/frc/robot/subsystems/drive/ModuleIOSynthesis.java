package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SynthesisDevices;

/**
 * One swerve module whose physics lives in Autodesk Synthesis instead of in this program.
 *
 * <p>{@link ModuleIOSim} integrates a motor model locally, so the robot can drive through walls and
 * other robots. Here the wheel and the steering hinge are rigid bodies in Synthesis's Jolt physics
 * world, shared with every other player's robot and the game pieces. This class only forwards
 * commands out and encoder readings back.
 *
 * <p>Synthesis motors take a velocity fraction, not a voltage (see {@link SynthesisDevices}), so
 * the drive is commanded open loop as {@code setpoint / kDriveMaxRadPerSec} and Synthesis's joint
 * servo does the tracking. {@link #kDriveMaxRadPerSec} must equal the wheel "max velocity"
 * configured for this robot in Synthesis.
 */
public class ModuleIOSynthesis implements ModuleIO {
    /** Wheel max velocity configured in Synthesis, rad/s at the wheel. 100 rad/s ~ 4.9 m/s. */
    public static final double kDriveMaxRadPerSec =
            DriveConstants.kMaxSpeedMetersPerSec / DriveConstants.kWheelRadiusMeters;

    /** Steering: hinge velocity fraction per radian of error. Saturates beyond 1/kTurnGain rad. */
    private static final double kTurnGain = 2.0;

    /** Flip if a wheel or hinge turns the wrong way for this robot model. */
    private static final double kDriveSign = 1.0;

    private static final double kTurnSign = 1.0;

    private final SynthesisDevices.Motor driveMotor;
    private final SynthesisDevices.Encoder driveEncoder;
    private final SynthesisDevices.Motor turnMotor;
    private final SynthesisDevices.Encoder turnEncoder;

    private double driveOutput = 0.0;
    private double turnOutput = 0.0;
    private Rotation2d turnSetpoint = null;

    public ModuleIOSynthesis(int index) {
        driveMotor = new SynthesisDevices.Motor("Drive", DriveConstants.kDriveMotorIds[index]);
        driveEncoder = new SynthesisDevices.Encoder("Drive", DriveConstants.kDriveMotorIds[index]);
        turnMotor = new SynthesisDevices.Motor("Turn", DriveConstants.kTurnMotorIds[index]);
        turnEncoder = new SynthesisDevices.Encoder("Turn", DriveConstants.kTurnMotorIds[index]);
    }

    private Rotation2d turnAngle() {
        return new Rotation2d(MathUtil.angleModulus(kTurnSign * turnEncoder.getPositionRad()));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        Rotation2d angle = turnAngle();

        // The steering loop has to run here each cycle; Synthesis only knows velocity requests.
        if (turnSetpoint != null) {
            double error = MathUtil.angleModulus(turnSetpoint.getRadians() - angle.getRadians());
            turnOutput = MathUtil.clamp(kTurnGain * error, -1.0, 1.0);
        }
        turnMotor.set(kTurnSign * turnOutput);
        driveMotor.set(kDriveSign * driveOutput);

        inputs.driveConnected = true;
        inputs.drivePositionMeters =
                kDriveSign * driveEncoder.getPositionRad() * DriveConstants.kWheelRadiusMeters;
        inputs.driveVelocityMetersPerSec =
                kDriveSign
                        * driveEncoder.getVelocityRadPerSec()
                        * DriveConstants.kWheelRadiusMeters;
        inputs.driveAppliedVolts = driveOutput * 12.0;

        inputs.turnConnected = true;
        inputs.absoluteEncoderConnected = true;
        inputs.turnPosition = angle;
        inputs.turnVelocityRadPerSec = kTurnSign * turnEncoder.getVelocityRadPerSec();
        inputs.turnAppliedVolts = turnOutput * 12.0;
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec) {
        driveOutput = velocityMetersPerSec / DriveConstants.kWheelRadiusMeters / kDriveMaxRadPerSec;
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveOutput = volts / 12.0;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnSetpoint = rotation;
    }

    @Override
    public void stop() {
        driveOutput = 0.0;
        turnOutput = 0.0;
        turnSetpoint = null;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        driveMotor.setBrakeMode(brake);
        turnMotor.setBrakeMode(brake);
    }
}
