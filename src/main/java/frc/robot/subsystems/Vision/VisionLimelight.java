package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Collection;

public class VisionLimelight {

    private static final double TAG_HEIGHT = Units.inchesToMeters(6.5);
    private static final double HEIGHT_OF_TAG_OFF_GROUND = Units.inchesToMeters(8.87);
    public static final double VERTICAL_RESOLUTION_IN_PIXELS = 800;
    public static final double VERTICAL_FIELD_OF_VIEW_IN_DEGREES = 56.2;
    public static final double LIMELIGHT_LATENCY_CONSTANT_SECONDS = 0.0;

    /**
     * When limelight tx was compared to robot rotation from the gyro, it was observed that they did not scale at the same rate. Assuming the Pigeon2 scales correctly, this means the limelight's angle scaling is incorrect. Because the difference scaled linearly, we found out the ratio between the limelight angle and robot angle and applied it to the tx angle.
     */
    public static final double LIMELIGHT_SCALAR_FACTOR = 38.12 / 35.99; // (38.0 / 30.0);

    private final NetworkTableEntry validEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry ledEntry;
    private final NetworkTableEntry cornerEntries; // Layout is {0x,0y,1x,1y,etc.}
    private final NetworkTableEntry throttleSetEntry;

    private final ArrayList<Pair<Double, Double>> cornerListPairs;

    private final String limelightName;
    private final CameraConfiguration.Location limelightLocation;
    private final Translation2d cameraToRobotCenter;
    private final double offsetBetweenAprilTagBottomAndCamera;
    private final Rotation2d limeLightYaw;
    private final double distanceScalarValue;

    public VisionLimelight(CameraConfiguration configuration) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        limeLightYaw = Rotation2d.fromRadians(configuration.LimelightMountingYawRadians);
        limelightName = configuration.getName();
        limelightLocation = configuration.getLocation();
        cameraToRobotCenter = configuration.getTranslationToRobotCenter();
        offsetBetweenAprilTagBottomAndCamera = HEIGHT_OF_TAG_OFF_GROUND - configuration.LimelightHeightOffsetMeters;
        distanceScalarValue = configuration.LimelightDistanceScalarValue;

        // Getting data from Network Tables
        validEntry = inst.getTable(limelightName).getEntry("tv");
        txEntry = inst.getTable(limelightName).getEntry("tx");
        tagIdEntry = inst.getTable(limelightName).getEntry("tid");
        ledEntry = inst.getTable(limelightName).getEntry("ledMode");
        cornerEntries = inst.getTable(limelightName).getEntry("tcornxy");
        throttleSetEntry = inst.getTable(limelightName).getEntry("throttle_set");

        cornerListPairs = new ArrayList<Pair<Double, Double>>();
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        inputs.horizontalAngleToTarget = Rotation2d.fromDegrees(txEntry.getDouble(0.0));
        inputs.hasTargets = validEntry.getDouble(0.0) == 1.0; // this was ty != 0 last year - double check
        inputs.tagId = (int) tagIdEntry.getDouble(-1);
        cornerListPairs.clear();
        var x = cornerEntries.getDoubleArray(new double[0]);
        if (x.length >= 8) {
            for (int i = 0; i < 7; i += 2) {
                cornerListPairs.add(new Pair<>(x[i], x[i + 1]));
            }
        }

        Logger.recordOutput(limelightName + "/Number of corners", cornerListPairs.size());

        if (inputs.hasTargets && cornerListPairs.size() >= 4) {
            inputs.targetHeight = calculateTargetHeightInPixels(cornerListPairs);
            inputs.angleEncompassingTag = calculateAngleEncompassingTagHeight(inputs.targetHeight);
            inputs.distanceToTagMeters = calculateDistanceToAprilTagInMetersUsingTrigMethod(inputs.angleEncompassingTag)
                    * distanceScalarValue;
        } else {
            inputs.targetHeight = -1;
            inputs.angleEncompassingTag = new Rotation2d();
            inputs.distanceToTagMeters = -1;
        }

        if (inputs.hasTargets && inputs.distanceToTagMeters != -1 && inputs.tagId > 0 && inputs.tagId <= 22) {
            inputs.robotPoseBasedOffDistanceCalcAndTagLocation =
                    calculateRobotPose(inputs.tagId, inputs.distanceToTagMeters, inputs.horizontalAngleToTarget);
        } else {
            inputs.robotPoseBasedOffDistanceCalcAndTagLocation = null;
        }
    }

    
    public void setLeds(boolean on) {
        ledEntry.setNumber(on ? 2 : 1);
    }

    
    public void setThrottleValue(int throttleValue) {
        throttleSetEntry.setNumber(throttleValue);
    }

    static double calculateTargetHeightInPixels(Collection<Pair<Double, Double>> sortedCorners) {
        return sortedCorners.stream().mapToDouble(Pair::getSecond).max().orElse(-1)
                - sortedCorners.stream().mapToDouble(Pair::getSecond).min().orElse(-1);
    }

    public static Collection<Pair<Double, Double>> sortCorners(Collection<Pair<Double, Double>> entries) {
        ArrayList<Pair<Double, Double>> finalCornerList = new ArrayList<Pair<Double, Double>>();

        for (int i = 0; i < 4; i++) {
            finalCornerList.add(Pair.of(0.0, 0.0));
        }

        double CentroidX =
                entries.stream().mapToDouble(Pair::getFirst).average().orElse(0.0);
        double CentroidY =
                entries.stream().mapToDouble(Pair::getSecond).average().orElse(0.0);

        for (int i = 0; i < 4; i++) {
            if (entries.stream().toList().get(i).getFirst() - CentroidX > 0) {
                if (entries.stream().toList().get(i).getSecond() - CentroidY > 0) {
                    finalCornerList.set(1, entries.stream().toList().get(i));
                } else {
                    finalCornerList.set(2, entries.stream().toList().get(i));
                }
            } else if (entries.stream().toList().get(i).getFirst() - CentroidX < 0) {
                if (entries.stream().toList().get(i).getSecond() - CentroidY < 0) {
                    finalCornerList.set(3, entries.stream().toList().get(i));
                } else {
                    finalCornerList.set(0, entries.stream().toList().get(i));
                }
            }
        }
        return finalCornerList;
    }

    static Rotation2d calculateAngleEncompassingTagHeight(double pixelHeight) {
        return Rotation2d.fromDegrees(pixelHeight * VERTICAL_FIELD_OF_VIEW_IN_DEGREES / VERTICAL_RESOLUTION_IN_PIXELS);
    }

    double calculateDistanceToAprilTagInMetersUsingTrigMethod(Rotation2d angle) {
        var tanOfTheta1 = angle.getTan();
        var sqrtTerm = Math.sqrt(Math.abs(Math.pow(TAG_HEIGHT, 2)
                - (4
                        * tanOfTheta1
                        * (offsetBetweenAprilTagBottomAndCamera + TAG_HEIGHT)
                        * tanOfTheta1
                        * offsetBetweenAprilTagBottomAndCamera)));
        return Math.abs((TAG_HEIGHT + sqrtTerm) / (2 * tanOfTheta1));
    }

    Pose2d calculateRobotPose(final int tagId, double distanceToTagMeters, Rotation2d horizontalAngleToTarget) {
        var tagPose = FieldConstants.getTagPose(tagId).toPose2d();

        var robotRotation =
                RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation();

        var robotRotationWithLimelightCorrection = robotRotation.plus(limeLightYaw);

        var scaledTx = Rotation2d.fromDegrees(
                -horizontalAngleToTarget.div(LIMELIGHT_SCALAR_FACTOR).getDegrees());

        var cameraToRobotCenter = this.cameraToRobotCenter.rotateBy(
                RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation());

        var angleToTag = scaledTx.plus(robotRotationWithLimelightCorrection);

        var translation = new Translation2d(distanceToTagMeters, angleToTag);
        var translatedPose = tagPose.getTranslation().minus(translation);

        var fieldRelativeRobotTranslation = translatedPose.plus(cameraToRobotCenter);

        var fieldRelativeChassisSpeeds = RobotState.getInstance().getRobotChassisSpeeds();

        var latencyCompensatedFieldRelativeTranslation = new Translation2d(
                fieldRelativeRobotTranslation.getX()
                        + (fieldRelativeChassisSpeeds.vxMetersPerSecond * LIMELIGHT_LATENCY_CONSTANT_SECONDS),
                fieldRelativeRobotTranslation.getY()
                        + (fieldRelativeChassisSpeeds.vyMetersPerSecond * LIMELIGHT_LATENCY_CONSTANT_SECONDS));

        var latencyCompensatedPose = new Pose2d(latencyCompensatedFieldRelativeTranslation, robotRotation);
        var uncompensatedPose = new Pose2d(fieldRelativeRobotTranslation, robotRotation);

        Logger.recordOutput(limelightName + "/latencyCompensatedPose", latencyCompensatedPose);
        Logger.recordOutput(limelightName + "/uncompensatedPose", uncompensatedPose);

        return latencyCompensatedPose;
    }

    
    public CameraConfiguration.Location getLimelightLocation() {
        return limelightLocation;
    }

    
    public String getLimelightName() {
        return limelightName;
    }
}  

