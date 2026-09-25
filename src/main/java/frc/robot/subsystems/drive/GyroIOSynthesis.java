package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SynthesisDevices;

/**
 * The chassis yaw as measured by Synthesis's physics world.
 *
 * <p>Unlike {@link GyroIOSim}, which integrates the rotation the modules asked for, this reports
 * what the chassis actually did -- including being spun by a collision with another robot.
 *
 * <p>Yaw comes from the true field heading Synthesis publishes (see {@link
 * SynthesisDevices.FieldPose}) rather than from Synthesis's integrating gyro sensor: in a match
 * that sensor drifted about 240 degrees while the robot bumped through fuel, where a real Pigeon 2
 * drifts a degree or two. The rate still comes from the gyro sensor.
 */
public class GyroIOSynthesis implements GyroIO {
    /** Synthesis reports yaw (rotation about the vertical axis) on its 'z' channel. */
    private static final char kYawAxis = 'z';

    /** Flip if the robot's heading turns the wrong way; WPILib wants counter-clockwise positive. */
    private static final double kYawSign = 1.0;

    private final SynthesisDevices.Gyro gyro =
            new SynthesisDevices.Gyro("Pigeon2", DriveConstants.kPigeonId);
    private final SynthesisDevices.FieldPose fieldPose = SynthesisDevices.fieldPose();
    private double offsetRad = 0.0;

    private double rawYawRad() {
        return fieldPose.placementCount() > 0
                ? fieldPose.headingRad()
                : kYawSign * Units.degreesToRadians(gyro.getAngleDeg(kYawAxis));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = new Rotation2d(rawYawRad() - offsetRad);
        inputs.yawVelocityRadPerSec =
                kYawSign * Units.degreesToRadians(gyro.getRateDegPerSec(kYawAxis));
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        offsetRad = rawYawRad() - yaw.getRadians();
    }
}
