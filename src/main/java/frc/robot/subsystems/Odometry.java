package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
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
  }

  /* Runs periodically (about once every 20 ms) */
  @Override
  public void periodic() {
    Logger.recordOutput("Odometry/Pose", getPose());
    Logger.recordOutput("Odometry/Heading", getHeading());

    // Odometry
    poseEstimator.update(getGyrometerHeading(), swerve.getModulePositions());

    LimelightHelpers.PoseEstimate pos_cev =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-cev");
    LimelightHelpers.PoseEstimate pos_uni =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-uni");

    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));

    if (pos_cev.tagCount > 0 && pos_cev.rawFiducials[0].ambiguity < 0.7) {
      poseEstimator.addVisionMeasurement(pos_cev.pose, pos_cev.timestampSeconds);
    }
    if (pos_uni.tagCount > 0 && pos_uni.rawFiducials[0].ambiguity < 0.7) {
      poseEstimator.addVisionMeasurement(pos_uni.pose, pos_cev.timestampSeconds);
    }
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
