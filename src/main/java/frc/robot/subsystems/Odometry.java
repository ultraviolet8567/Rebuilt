package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public Odometry(Swerve swerve) {
        System.out.println("[Init] Creating Odometry");

        this.swerve = swerve;

        /* Gyro */
        gyro = new Pigeon2(30);
        gyro.reset();

        /* Odometry */
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kDriveKinematics,
                        gyro.getRotation2d(),
                        swerve.getModulePositions(),
                        new Pose2d());

        //poseEstimator.setVisionMeasurementStdDevs(new Matrix<>())

        LimelightHelpers.setCameraPose_RobotSpace(
                "limelight-" + OdometryConstants.kActiveCamera,
                OdometryConstants.kTranslationOffset.getX(),
                OdometryConstants.kTranslationOffset.getY(),
                OdometryConstants.kTranslationOffset.getZ(),
                OdometryConstants.kRotationOffset.getX(),
                OdometryConstants.kRotationOffset.getY(),
                OdometryConstants.kRotationOffset.getZ());
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

        // if (LimelightHelpers.validPoseEstimate(visionPose))
        //    poseEstimator.addVisionMeasurement(visionPose.pose, visionPose.timestampSeconds);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
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
    }

    public void resetHeading() {
        gyro.reset();
        // gyro.setYaw(((DriverStation.getAlliance().get() == Alliance.Blue) ? 0 : 180));
    }

    public Pose2d getHub() {
        return (DriverStation.getAlliance().get() == Alliance.Blue
                ? FieldConstants.kBlueHub
                : FieldConstants.kRedHub);
    }

    public double distToHub() {
        return getPose().getTranslation().getDistance(getHub().getTranslation());
    }
}
