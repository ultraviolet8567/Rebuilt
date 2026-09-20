package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

public class Odometry extends SubsystemBase {
    private Swerve swerve;
    private SwerveDrivePoseEstimator poseEstimator;

    private Pigeon2 gyro;

    // Desktop simulation only: the Pigeon's simulated raw yaw, integrated from the chassis
    // rotation rate. The Pigeon2's own setYaw()/reset() offsets still apply on top of this.
    private Pigeon2SimState gyroSim;
    private double simYawDegrees = 0.0;

    public Odometry(Swerve swerve) {
        System.out.println("[Init] Creating Odometry");

        this.swerve = swerve;

        /* Gyro */
        gyro = new Pigeon2(30);
        if (RobotBase.isSimulation()) {
            gyroSim = gyro.getSimState();
            gyroSim.setRawYaw(0.0);
        }
        resetHeading();

        /* Odometry */
        // Seed the estimator's heading FROM the gyro so its internal gyro offset is zero.
        // resetHeading() above just set the yaw to 0 (Blue) or 180 (Red); if the estimator
        // were seeded with a 0-degree pose while the gyro already reads 180, it would carry a
        // permanent 180-degree offset and integrate wheel motion in the wrong direction for
        // Red in teleop (an auto's resetPose() hides this, a practice session does not).
        Rotation2d initialHeading = gyro.getRotation2d();
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kDriveKinematics,
                        initialHeading,
                        swerve.getModulePositions(),
                        new Pose2d(0.0, 0.0, initialHeading));

        // poseEstimator.setVisionMeasurementStdDevs(new Matrix<>())

        LimelightHelpers.setCameraPose_RobotSpace(
                "limelight-" + OdometryConstants.kActiveCamera,
                OdometryConstants.kTranslationOffset.getX(),
                OdometryConstants.kTranslationOffset.getY(),
                OdometryConstants.kTranslationOffset.getZ(),
                OdometryConstants.kCameraRollDegrees,
                OdometryConstants.kCameraPitchDegrees,
                OdometryConstants.kCameraYawDegrees);

        // x/y from the Limelight are trusted moderately. Rotation is NOT: with MegaTag2 the
        // vision rotation is derived from the gyro we already feed it, so fusing it tightly
        // just lets vision fight the gyro. A huge stddev means "ignore vision rotation".
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
    }

    /* Runs periodically (about once every 20 ms) */
    @Override
    public void periodic() {
        Logger.recordOutput("Odometry/Pose", getPose());
        Logger.recordOutput("Odometry/Heading", getHeading());
        Logger.recordOutput("Odometry/GyroHeading", getGyrometerHeading());

        // Odometry
        poseEstimator.update(getGyrometerHeading(), swerve.getModulePositions());

        /*
        double[] visionRawPose =
                NetworkTableInstance.getDefault()
                        .getTable("limelight-" + OdometryConstants.kActiveCamera)
                        .getEntry("botpose")
                        .getDoubleArray(new double[6]);
        */

        LimelightHelpers.SetRobotOrientation(
                "limelight-" + OdometryConstants.kActiveCamera,
                getGyrometerHeading().getDegrees(),
                0,
                0,
                0,
                0,
                0);

        PoseEstimate visionPose =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                        "limelight-" + OdometryConstants.kActiveCamera);

        // Logger.recordOutput("Odometry/visionPose", visionPose.pose);

        Logger.recordOutput("Odometry/AngleToHub", angleToHub());
        Logger.recordOutput("Odometry/GyrometerAngle", getGyrometerHeading().getRadians());
        Logger.recordOutput("Odometry/GyrometerRots", getGyrometerHeading().getRotations());

        Logger.recordOutput("Odometry/DistToHub", distToHub());

        Logger.recordOutput("Odometry/BlueHub", FieldConstants.kBlueHub);
        Logger.recordOutput("Odometry/RedHub", FieldConstants.kRedHub);

        // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(9999999, 9999999, 9999999));

        if (LimelightHelpers.validPoseEstimate(visionPose)) {
            poseEstimator.addVisionMeasurement(visionPose.pose, visionPose.timestampSeconds);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return getGyrometerHeading();
    }

    public Rotation2d angleToHub() {
        return (getPose().getTranslation().minus(getHub().getTranslation()))
                .getAngle()
                .plus(new Rotation2d(Math.PI));

        // return (higherAngle.minus(getGyrometerHeading()).getRadians()
        //                > getGyrometerHeading().minus(lowerAngle).getRadians()
        //        ? lowerAngle
        //        : higherAngle);
    }

    public Rotation2d getGyrometerHeading() {
        return gyro.getRotation2d();
    }

    public void setGyroYaw(Rotation2d yaw) {
        gyro.setYaw(yaw.getDegrees());
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), swerve.getModulePositions(), pose);
        if (RobotBase.isSimulation()) {
            swerve.resetSimTruePose(pose);
        }
    }

    // ------------------------------------------------------------------------------------
    // Desktop simulation
    // ------------------------------------------------------------------------------------

    /**
     * Called by the CommandScheduler each loop, only on a desktop. Swerve.simulationPeriodic() has
     * already advanced the wheels this loop (Swerve is registered first), so the chassis rotation
     * rate we read here is current.
     */
    @Override
    public void simulationPeriodic() {
        double omegaRadPerSec = swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond;
        simYawDegrees += Math.toDegrees(omegaRadPerSec * Constants.kLoopPeriodSecs);
        gyroSim.setRawYaw(simYawDegrees);
        gyroSim.setAngularVelocityZ(Math.toDegrees(omegaRadPerSec));

        Logger.recordOutput("Odometry/SimRawYawDegrees", simYawDegrees);
    }

    public void resetHeading() {
        if (DriverStation.getAlliance().isPresent()) {
            gyro.setYaw(((DriverStation.getAlliance().get() == Alliance.Blue) ? 0 : 180));
        } else {
            gyro.reset();
        }
    }

    public Pose2d getHub() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) {
            return FieldConstants.kRedHub;
        }
        return FieldConstants.kBlueHub;
    }

    public double distToHub() {
        return getPose().getTranslation().getDistance(getHub().getTranslation());
    }
}
