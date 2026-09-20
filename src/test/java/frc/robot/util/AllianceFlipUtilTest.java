package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TestRobot;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class AllianceFlipUtilTest {
    @BeforeAll
    static void boot() {
        TestRobot.init();
    }

    @AfterAll
    static void done() {
        TestRobot.shutdown();
    }

    @Test
    void noAllianceMeansNoFlip() {
        TestRobot.clearAlliance();
        assertFalse(AllianceFlipUtil.shouldFlip());
        assertEquals(1.5, AllianceFlipUtil.apply(1.5));
    }

    @Test
    void blueDoesNotFlip() {
        TestRobot.setAlliance(Alliance.Blue);
        assertFalse(AllianceFlipUtil.shouldFlip());
        Pose2d p = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
        assertEquals(p, AllianceFlipUtil.apply(p));
    }

    @Test
    void redMirrorsAcrossTheFieldCenterline() {
        TestRobot.setAlliance(Alliance.Red);
        assertTrue(AllianceFlipUtil.shouldFlip());
        assertEquals(-2.0, AllianceFlipUtil.apply(2.0));
        assertEquals(new Translation2d(-2, 3), AllianceFlipUtil.apply(new Translation2d(2, 3)));
        // A heading of 0 (facing +x) becomes 180 (facing -x); 30 becomes 150.
        assertEquals(180, AllianceFlipUtil.apply(Rotation2d.fromDegrees(0)).getDegrees(), 1e-9);
        assertEquals(150, AllianceFlipUtil.apply(Rotation2d.fromDegrees(30)).getDegrees(), 1e-9);
    }
}
