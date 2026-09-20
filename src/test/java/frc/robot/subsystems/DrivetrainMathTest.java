package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.TestRobot;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class DrivetrainMathTest {
    private static Swerve swerve;
    private static Odometry odometry;

    @BeforeAll
    static void boot() {
        TestRobot.init();
        Lights.getInstance();
        swerve = new Swerve(ModuleConstants.kDriveMotorGearing);
        odometry = new Odometry(swerve);
    }

    @AfterAll
    static void done() {
        TestRobot.shutdown();
    }

    @Test
    void hubFollowsTheAllianceAndDefaultsToBlue() {
        assertEquals(FieldConstants.kBlueHub, odometry.getHub(), "no alliance -> blue hub");
        TestRobot.setAlliance(Alliance.Red);
        assertEquals(FieldConstants.kRedHub, odometry.getHub());
        TestRobot.setAlliance(Alliance.Blue);
        assertEquals(FieldConstants.kBlueHub, odometry.getHub());
    }

    @Test
    void distanceAndAngleToHubComeFromThePose() {
        TestRobot.setAlliance(Alliance.Blue);
        Pose2d hub = FieldConstants.kBlueHub;
        // 3 m directly in -x from the hub, facing +x: the hub is straight ahead.
        odometry.resetPose(new Pose2d(hub.getX() - 3.0, hub.getY(), new Rotation2d()));
        assertEquals(3.0, odometry.distToHub(), 1e-6);
        assertEquals(0.0, odometry.angleToHub().getDegrees(), 1e-6);
        // 2 m in +y from the hub: the hub is at -90 degrees (to the right of +x).
        odometry.resetPose(new Pose2d(hub.getX(), hub.getY() + 2.0, new Rotation2d()));
        assertEquals(2.0, odometry.distToHub(), 1e-6);
        assertEquals(-90.0, odometry.angleToHub().getDegrees(), 1e-6);
    }

    @Test
    void demoModeReducesTheDrivetrainSpeedLimit() {
        Lights.getInstance().isDemo = false;
        assertEquals(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, swerve.getMaxSpeed());
        Lights.getInstance().isDemo = true;
        assertEquals(DriveConstants.kDemoTeleDriveMaxSpeedMetersPerSecond, swerve.getMaxSpeed());
        assertEquals(2.75, swerve.getMaxSpeed(), 0.01, "demo speed stays at the field-used value");
        Lights.getInstance().isDemo = false;
    }
}
