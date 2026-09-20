package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.TestRobot;
import frc.robot.subsystems.Intake.Funnel;
import frc.robot.subsystems.Intake.Pivot;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Kicker;
import frc.robot.subsystems.Storage.Indexer;
import frc.robot.subsystems.Swerve;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Command wiring and behavior, run against the real RobotContainer in the HAL simulator. */
public class CommandBehaviorTest {
    private static RobotContainer container;
    private static Swerve swerve;
    private static Odometry odometry;
    private static Flywheel flywheel;
    private static Hood hood;
    private static Kicker kicker;
    private static Pivot pivot;
    private static Funnel funnel;
    private static Indexer indexer;

    @BeforeAll
    static void boot() {
        TestRobot.init();
        container = new RobotContainer();
        swerve = container.getSwerve();
        odometry = container.getOdometry();
        flywheel = container.getShooter().getFlywheel();
        hood = container.getShooter().getHood();
        kicker = container.getShooter().getKicker();
        pivot = container.getIntake().getPivot();
        funnel = container.getIntake().getFunnel();
        indexer = container.getStorage().getIndexer();
    }

    @AfterAll
    static void done() {
        TestRobot.shutdown();
    }

    private static void requires(Command c, Object... subsystems) {
        for (Object s : subsystems) {
            assertTrue(
                    c.getRequirements().contains(s),
                    c.getClass().getSimpleName() + " must require " + s.getClass().getSimpleName());
        }
    }

    @Test
    void everyCommandDeclaresItsSubsystems() {
        requires(new Shoot(flywheel, 3000), flywheel);
        requires(new DirectShoot(flywheel), flywheel);
        requires(new DirectHood(hood), hood);
        requires(new SetHood(hood, false), hood);
        requires(new SetPivot(pivot, 1.0), pivot);
        requires(new Feed(pivot), pivot);
        requires(new ManualKicker(kicker), kicker);
        requires(new RunKicker(flywheel, kicker), kicker);
        requires(new SpinIntake(funnel, false), funnel);
        requires(new SpinIndexer(indexer, false), indexer);
        requires(new CalculatedShoot(flywheel, odometry, indexer), flywheel, indexer);
        requires(new Shuffle(flywheel, hood, indexer), flywheel, hood, indexer);
        requires(
                new ManualTeleOp(
                        swerve,
                        odometry,
                        () -> 0.0,
                        () -> 0.0,
                        () -> 0.0,
                        () -> false,
                        () -> false),
                swerve);
        requires(
                new DriftTeleOp(
                        swerve,
                        odometry,
                        () -> 0.0,
                        () -> 0.0,
                        () -> 0.0,
                        () -> false,
                        () -> false),
                swerve);
    }

    @Test
    void conflictingPivotCommandsCancelEachOther() {
        Command feed = new Feed(pivot);
        Command set = new SetPivot(pivot, IntakeConstants.kPivotMiddle);
        feed.schedule();
        TestRobot.loop(2);
        assertTrue(feed.isScheduled());
        set.schedule();
        TestRobot.loop(2);
        assertFalse(feed.isScheduled(), "SetPivot must interrupt Feed (shared pivot requirement)");
        CommandScheduler.getInstance().cancelAll();
    }

    @Test
    void feedOscillatesThePivotTarget() {
        Feed feed = new Feed(pivot);
        feed.initialize();
        for (int i = 0; i < 10; i++) feed.execute();
        assertEquals(IntakeConstants.kPivotUpper, pivot.getTargetPosition(), 1e-9);
        for (int i = 0; i < 20; i++) feed.execute(); // count = 30 -> 30 % 50 > 25
        assertEquals(IntakeConstants.kPivotLower, pivot.getTargetPosition(), 1e-9);
        feed.end(false);
        assertEquals(IntakeConstants.kPivotLower, pivot.getTargetPosition(), 1e-9);
    }

    @Test
    void calculatedShootRecomputesFromLiveDistance() {
        Command cmd = new CalculatedShoot(flywheel, odometry, indexer);
        odometry.resetPose(new Pose2d(0, 0, new Rotation2d()));
        cmd.schedule();
        TestRobot.loop(3);
        double far = flywheel.getTargetVelocity();
        odometry.resetPose(new Pose2d(2, 4, new Rotation2d())); // much closer to the blue hub
        TestRobot.loop(3);
        double near = flywheel.getTargetVelocity();
        assertNotEquals(far, near, "target must follow the robot while the trigger is held");
        assertEquals(flywheel.calculateTargetVelocity(odometry.distToHub()), near, 1e-6);
        cmd.cancel();
        TestRobot.loop(2);
    }

    @Test
    void shuffleRaisesHoodThenLowersItWhenReleased() {
        Command cmd = new Shuffle(flywheel, hood, indexer);
        cmd.schedule();
        TestRobot.loop(2);
        assertEquals(ShooterConstants.kHoodUpper, hood.getTargetPosition(), 1e-9);
        cmd.cancel();
        TestRobot.loop(2);
        assertEquals(ShooterConstants.kHoodLower, hood.getTargetPosition(), 1e-9);
    }

    @Test
    void setHoodNudgesTheTargetAndClampsAtTheLimits() {
        hood.setTargetPosition(ShooterConstants.kHoodLower);
        SetHood up = new SetHood(hood, false);
        for (int i = 0; i < 5; i++) up.execute();
        assertEquals(ShooterConstants.kHoodLower + 0.05, hood.getTargetPosition(), 1e-9);
        for (int i = 0; i < 100; i++) up.execute();
        assertEquals(ShooterConstants.kHoodUpper, hood.getTargetPosition(), 1e-9, "clamped");
        hood.setTargetPosition(ShooterConstants.kHoodLower);
    }

    @Test
    void kickerRunsOnlyOnceTheFlywheelIsAtSpeed() {
        // RunKicker is the kicker's default command, so it is already running.
        flywheel.stop();
        TestRobot.loop(150);
        assertFalse(kicker.isRunning(), "flywheel stopped -> kicker off");
        flywheel.start(3000);
        TestRobot.loop(3);
        assertFalse(kicker.isRunning(), "flywheel still spinning up -> kicker off");
        TestRobot.loop(150);
        assertTrue(flywheel.atVelocity(3000));
        assertTrue(kicker.isRunning(), "flywheel at speed -> kicker on");
        flywheel.stop();
        TestRobot.loop(150);
        assertFalse(kicker.isRunning());
    }
}
