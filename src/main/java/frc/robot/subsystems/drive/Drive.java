package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.Alerts;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The swerve drivetrain, its gyro and its pose estimate.
 *
 * <p>Odometry used to be a separate subsystem holding a reference to the drivetrain, which created
 * a fixed ordering requirement (Swerve had to be registered before Odometry so that the gyro
 * simulation read a rotation rate from the current loop rather than the previous one) that nothing
 * in the code enforced. Pose estimation is not a mechanism; it is what the drivetrain knows about
 * itself, so it lives here. All 17 corpus robots that run a pose estimator put it on the drive
 * subsystem.
 */
public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];

    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };

    /** Simulation only: dead-reckoned truth, to measure how far the pose estimate drifts. */
    private Pose2d simTruePose = new Pose2d();

    private final Alert gyroDisconnected =
            Alerts.create("Gyro disconnected -- driving robot-relative", AlertType.kError);

    public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(fl, 0);
        modules[1] = new Module(fr, 1);
        modules[2] = new Module(bl, 2);
        modules[3] = new Module(br, 3);

        poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kKinematics,
                        rawGyroRotation,
                        lastModulePositions,
                        new Pose2d());

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::runVelocity,
                DriveConstants.kPathFollower,
                DriveConstants.kPathPlannerConfig,
                () -> RobotState.getInstance().isRedAlliance(),
                this);
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (Module module : modules) {
            module.periodic();
        }

        gyroDisconnected.set(!gyroInputs.connected && Constants.currentMode != Constants.Mode.SIM);

        // Feed the pose estimator. When the gyro drops out mid-match, fall back on integrating
        // the wheel-derived rotation rather than freezing the heading at its last good value --
        // a frozen heading silently corrupts every field-relative command that follows.
        SwerveModulePosition[] positions = getModulePositions();
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            deltas[i] =
                    new SwerveModulePosition(
                            positions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                            positions[i].angle);
            lastModulePositions[i] = positions[i];
        }

        if (gyroInputs.connected) {
            rawGyroRotation = gyroInputs.yawPosition;
        } else {
            Twist2d twist = DriveConstants.kKinematics.toTwist2d(deltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }
        poseEstimator.update(rawGyroRotation, positions);

        Logger.recordOutput("Drive/Pose", getPose());
        Logger.recordOutput("Drive/MeasuredStates", getModuleStates());
        Logger.recordOutput("Drive/ChassisSpeeds", getChassisSpeeds());
        Logger.recordOutput("Drive/MaxSpeed", getMaxLinearSpeedMetersPerSec());
        Logger.recordOutput("Drive/DistanceToHub", getDistanceToHub());
        Logger.recordOutput("Drive/AngleToHub", getAngleToHub());
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds speeds = getChassisSpeeds();
        double dt = Constants.kLoopPeriodSecs;
        simTruePose =
                simTruePose.exp(
                        new Twist2d(
                                speeds.vxMetersPerSecond * dt,
                                speeds.vyMetersPerSecond * dt,
                                speeds.omegaRadiansPerSecond * dt));
        Logger.recordOutput("Drive/SimTruePose", simTruePose);
    }

    // ------------------------------------------------------------------ commands in

    /**
     * Drive at the requested chassis speeds.
     *
     * <p>{@link ChassisSpeeds#discretize} is the important addition. Swerve kinematics assume the
     * robot travels in a straight line for the whole 20 ms loop, but when it is translating and
     * rotating at the same time the real path is an arc. Discretising corrects for that; without it
     * the robot drifts perpendicular to its motion whenever the driver turns while moving, and the
     * odometry believes the drift did not happen. 18 of the 42 corpus robots discretise.
     */
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discrete = ChassisSpeeds.discretize(speeds, Constants.kLoopPeriodSecs);
        SwerveModuleState[] setpoints = DriveConstants.kKinematics.toSwerveModuleStates(discrete);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, getMaxLinearSpeedMetersPerSec());

        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpoints[i]);
        }

        Logger.recordOutput("Drive/SetpointStates", setpoints);
        Logger.recordOutput("Drive/SetpointSpeeds", discrete);
    }

    /** Stop the drivetrain, leaving the wheels where they are. */
    public void stop() {
        for (Module module : modules) {
            module.stop();
        }
    }

    /**
     * Point the wheels into an X so the robot cannot be pushed.
     *
     * <p>The previous {@code lockWheels()} sent 45, 135, -45, 45 degrees -- the back-left and
     * back-right wheels were both at 45, which is a parallelogram, not an X, and the robot could
     * still be shoved sideways. Deriving the angles from the module positions makes the shape
     * correct by construction.
     */
    public void stopWithX() {
        for (int i = 0; i < 4; i++) {
            modules[i].runTurnOnly(DriveConstants.kModuleTranslations[i].getAngle());
        }
    }

    public void runCharacterization(double volts) {
        for (Module module : modules) {
            module.runCharacterization(volts);
        }
    }

    public void setBrakeMode(boolean brake) {
        for (Module module : modules) {
            module.setBrakeMode(brake);
        }
    }

    // ------------------------------------------------------------------ state out

    @AutoLogOutput(key = "Drive/ModuleStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kKinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Top speed, halved-ish in demo mode. Both limits scale together so handling is unchanged. */
    public double getMaxLinearSpeedMetersPerSec() {
        return RobotState.getInstance().isDemoMode()
                ? DriveConstants.kMaxSpeedMetersPerSec * DriveConstants.kDemoScaleFactor
                : DriveConstants.kMaxSpeedMetersPerSec;
    }

    public double getMaxAngularSpeedRadPerSec() {
        return RobotState.getInstance().isDemoMode()
                ? DriveConstants.kMaxAngularSpeedRadPerSec * DriveConstants.kDemoScaleFactor
                : DriveConstants.kMaxAngularSpeedRadPerSec;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Raw gyro yaw, before any pose-estimator correction. */
    public Rotation2d getGyroRotation() {
        return rawGyroRotation;
    }

    public double getYawVelocityRadPerSec() {
        return gyroInputs.yawVelocityRadPerSec;
    }

    public boolean isGyroConnected() {
        return gyroInputs.connected;
    }

    public Pose2d getSimTruePose() {
        return simTruePose;
    }

    public boolean allModulesConnected() {
        for (Module module : modules) {
            if (!module.isConnected()) {
                return false;
            }
        }
        return true;
    }

    // ------------------------------------------------------------------ pose

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        simTruePose = pose;
    }

    /** Declare the robot to be facing away from our alliance wall. */
    public void resetHeading() {
        Rotation2d heading =
                RobotState.getInstance().isRedAlliance()
                        ? Rotation2d.fromDegrees(180)
                        : Rotation2d.kZero;
        gyroIO.setYaw(heading);
        rawGyroRotation = heading;
        resetPose(new Pose2d(getPose().getTranslation(), heading));
    }

    /** Fuse a vision pose. Standard deviations come from the vision subsystem, per frame. */
    public void addVisionMeasurement(
            Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    }

    /** Wide-open default, replaced per measurement by {@link #addVisionMeasurement}. */
    public void setDefaultVisionStdDevs() {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
    }

    // ------------------------------------------------------------------ aiming

    public Translation2d getHubTranslation() {
        return FieldConstants.hubTranslation(RobotState.getInstance().isRedAlliance());
    }

    public double getDistanceToHub() {
        return getPose().getTranslation().getDistance(getHubTranslation());
    }

    /**
     * Field-relative heading that points the shooter at the hub.
     *
     * <p>Returned as a {@link Rotation2d} and consumed by a continuous-input controller, so the
     * wrap-around case is handled by WPILib rather than by the four hand-written branches the old
     * {@code DriftTeleOp} used.
     */
    public Rotation2d getAngleToHub() {
        return getHubTranslation().minus(getPose().getTranslation()).getAngle();
    }
}
