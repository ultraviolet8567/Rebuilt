package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.TestRobot;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** AutoChooser without a drivetrain: PathPlanner gets dummy pose/speed suppliers. */
public class AutoChooserTest {
    private static AutoChooser chooser;

    @BeforeAll
    static void boot() {
        TestRobot.init();
        AutoBuilder.configure(
                Pose2d::new,
                pose -> {},
                ChassisSpeeds::new,
                speeds -> {},
                AutoConstants.kHolonomicController,
                DriveConstants.kRobotConfig,
                () -> false,
                new SubsystemBase() {});
        chooser = new AutoChooser();
    }

    @AfterAll
    static void done() {
        TestRobot.shutdown();
    }

    private static void select(String side, String driveOut) {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("Shuffleboard/Main/Side").getEntry("selected").setString(side);
        nt.getTable("Shuffleboard/Main/Drive Out").getEntry("selected").setString(driveOut);
        nt.flushLocal();
        for (int i = 0; i < 5; i++) {
            Shuffleboard.update();
            chooser.periodic();
        }
    }

    @Test
    void defaultSelectionIsNotARoutineAndDoesNotCrash() {
        // Default is "" + " " + "" = " ", which is not an auto. This used to NPE.
        assertNull(chooser.getSelectedAuto());
        assertEquals(new Pose2d(), chooser.getAutoStartingPose());
        chooser.periodic();
    }

    @Test
    void nonexistentCombinationIsHandled() {
        select("Left", "Drive Out"); // "Drive Out Left" exists; try one that does not
        select("Right", "Drive Out"); // there is no "Drive Out Right" auto
        assertEquals("Drive Out Right", chooser.getAutoCommandName());
        assertNull(chooser.getSelectedAuto());
        assertEquals(new Pose2d(), chooser.getAutoStartingPose());
    }

    @Test
    void realCombinationLoadsTheRoutine() {
        select("Middle", "Shoot");
        assertEquals("Shoot Middle", chooser.getAutoCommandName());
        assertNotNull(
                chooser.getSelectedAuto(), "Shoot Middle should resolve to a PathPlannerAuto");
        Pose2d start = chooser.getAutoStartingPose();
        assertTrue(start.getX() > 0 || start.getY() > 0, "starting pose should come from the path");
    }
}
