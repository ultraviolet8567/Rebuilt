package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ManualTeleOp;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Boots the robot as the RED alliance and checks the alliance-dependent behavior. */
public class RedAllianceBootTest {
    private static Swerve swerve;
    private static Odometry odometry;

    @BeforeAll
    static void boot() {
        TestRobot.init(Alliance.Red);
        RobotContainer container = new RobotContainer();
        swerve = container.getSwerve();
        odometry = container.getOdometry();
        TestRobot.loop(10); // let the Pigeon signal pick up the boot-time yaw
    }

    @AfterAll
    static void done() {
        TestRobot.shutdown();
    }

    @Test
    void redBootsFacingOneEighty() {
        double yaw = MathUtil.inputModulus(odometry.getGyrometerHeading().getDegrees(), -180, 180);
        assertEquals(180, Math.abs(yaw), 2.0, "resetHeading() sets 180 for Red");
    }

    @Test
    void redUsesTheRedHub() {
        assertEquals(FieldConstants.kRedHub, odometry.getHub());
    }

    @Test
    void stickForwardDrivesAwayFromTheRedWall() {
        // Field-oriented "forward" for Red is toward -x (away from the red wall at high x).
        // Before the double-flip fix, Red used the wrong frame.
        Command drive =
                new ManualTeleOp(
                        swerve,
                        odometry,
                        () -> 1.0,
                        () -> 0.0,
                        () -> 0.0,
                        () -> false,
                        () -> false);
        double startX = odometry.getPose().getX();
        drive.schedule();
        TestRobot.loop(100);
        drive.cancel();
        double dx = odometry.getPose().getX() - startX;
        assertTrue(dx < -0.5, "Red stick-forward should move toward -x, moved " + dx);
        assertEquals(0.0, odometry.getPose().getY(), 0.3);
    }
}
