package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.LimelightHelpers;
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

        LimelightHelpers.setCameraPose_RobotSpace(
                OdometryConstants.kActiveCamera,
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

        double[] visionRawPose =
                NetworkTableInstance.getDefault()
                        .getTable("limelight-"+OdometryConstants.kActiveCamera)
                        .getEntry("botpose")
                        .getDoubleArray(new double[6]);
        Pose2d visionPose =
                new Pose2d(visionRawPose[0], visionRawPose[1], new Rotation2d(visionRawPose[4]));
        
        Logger.recordOutput("visionPose", visionPose);

        poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }


    public Rotation2d getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
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

    public void resetGyrometerHeading() {
        gyro.reset();
    }
}
