package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.SimBattery;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * Boots the whole robot in desktop simulation -- no hardware, no Driver Station app -- and drives
 * the simulated mechanisms to check that the physics models, the simulated sensors and the control
 * code agree with each other. Run with {@code ./gradlew test}.
 *
 * <p>The tests share one RobotContainer because motor controllers can only be created once per CAN
 * ID per process, so they run in a fixed order.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class SimulationSmokeTest {
    private static RobotContainer container;
    private static Drive drive;
    private static Flywheel flywheel;
    private static Hood hood;
    private static Pivot pivot;

    @BeforeAll
    static void bootRobot() {
        assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");

        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();

        // AdvantageKit is deliberately NOT started here -- it refuses to run outside a
        // LoggedRobot. Every Logger call is guarded by an "is running" check, so they no-op.

        // Same construction path as Robot(): this also proves the robot boots with NO alliance
        // assigned, which used to crash in the old Odometry.resetHeading().
        container = new RobotContainer();
        assertNotNull(container);

        drive = container.getDrive();
        flywheel = container.getShooter().getFlywheel();
        hood = container.getShooter().getHood();
        pivot = container.getIntake().getPivot();
    }

    @AfterAll
    static void shutdown() {
        CommandScheduler.getInstance().cancelAll();
        HAL.shutdown();
    }

    /**
     * One 20 ms robot loop.
     *
     * <p>No {@code Timer.delay} any more: the simulated gyro is a plain integrator rather than a
     * Phoenix sim state published by a background thread, so the loop is deterministic and the
     * suite runs in real CPU time rather than wall-clock time.
     */
    private static void loop(int cycles) {
        for (int i = 0; i < cycles; i++) {
            DriverStationSim.notifyNewData();
            DriverStation.refreshData();
            CommandScheduler.getInstance().run();
            SimBattery.update();
        }
    }

    /**
     * Run a command that owns the drivetrain so the joystick default command stays out of the way.
     */
    private static Command driveCommand(ChassisSpeeds speeds) {
        return new RunCommand(() -> drive.runVelocity(speeds), drive);
    }

    // ------------------------------------------------------------------ pure logic

    /**
     * The reason the range table exists. The old quadratic peaked at 4.96 m and fell off after
     * that, so a shot from 7 m was commanded a lower velocity than one from 5 m.
     */
    @Test
    @Order(0)
    void shooterRangeTableIsMonotonic() {
        double previous = -1;
        for (double d = 1.0; d <= 9.0; d += 0.25) {
            double rpm = Flywheel.rpmForDistance(d);
            assertTrue(
                    rpm >= previous,
                    "velocity must never decrease with distance; at "
                            + d
                            + " m got "
                            + rpm
                            + " after "
                            + previous);
            previous = rpm;
        }
    }

    /** Inside the calibrated band the table reproduces the team's own fitted values. */
    @Test
    @Order(0)
    void shooterRangeTableMatchesCalibratedFit() {
        for (double d : new double[] {2.0, 3.0, 4.0}) {
            double fitted = 1310.13 + 1270.33 * d - 128.09 * d * d;
            assertEquals(fitted, Flywheel.rpmForDistance(d), 5.0, "distance " + d);
        }
    }

    /** Outside the table it clamps rather than extrapolating into nonsense. */
    @Test
    @Order(0)
    void shooterRangeTableClampsOutsideItsRange() {
        assertEquals(Flywheel.rpmForDistance(1.5), Flywheel.rpmForDistance(0.2), 1e-6);
        assertEquals(Flywheel.rpmForDistance(8.0), Flywheel.rpmForDistance(30.0), 1e-6);
    }

    // ------------------------------------------------------------------ boot

    @Test
    @Order(1)
    void encodersAgreeAtBoot() {
        loop(5);
        assertEquals(
                hood.getAngleRad(),
                ShooterConstants.kHoodLowerRad,
                0.02,
                "hood should boot at its stowed angle");
        assertEquals(
                pivot.getAngleRad(),
                IntakeConstants.kPivotStowedRad,
                0.02,
                "pivot should boot stowed");
    }

    @Test
    @Order(1)
    void allSimulatedDevicesReportConnected() {
        loop(5);
        assertTrue(drive.allModulesConnected(), "modules");
        assertTrue(drive.isGyroConnected(), "gyro");
    }

    // ------------------------------------------------------------------ drivetrain

    @Test
    @Order(2)
    void drivingForwardMovesTheRobot() {
        Command cmd = driveCommand(new ChassisSpeeds(1.0, 0.0, 0.0));
        cmd.schedule();
        loop(100); // 2 seconds
        cmd.cancel();

        for (SwerveModuleState s : drive.getModuleStates()) {
            assertTrue(
                    s.speedMetersPerSecond > 0.5,
                    "module should be moving forward, got " + s.speedMetersPerSecond);
            assertEquals(0.0, s.angle.getRadians(), 0.15, "wheels should point straight ahead");
        }

        double odoX = drive.getPose().getX();
        double trueX = drive.getSimTruePose().getX();
        assertTrue(odoX > 1.0 && odoX < 2.5, "odometry x = " + odoX);
        assertEquals(trueX, odoX, 0.2, "odometry should track sim ground truth");
        assertEquals(0.0, drive.getPose().getY(), 0.2, "no sideways drift");
    }

    @Test
    @Order(3)
    void modulesCanSteer() {
        Command cmd = driveCommand(new ChassisSpeeds(0.0, 1.0, 0.0)); // strafe left = wheels at 90
        cmd.schedule();
        loop(75);
        cmd.cancel();

        for (SwerveModuleState s : drive.getModuleStates()) {
            assertEquals(
                    Math.PI / 2,
                    Math.abs(s.angle.getRadians()),
                    0.15,
                    "wheel should have steered to +/-90 deg");
        }
    }

    @Test
    @Order(4)
    void rotatingUpdatesTheSimulatedGyro() {
        Rotation2d before = drive.getGyroRotation();
        Command cmd = driveCommand(new ChassisSpeeds(0.0, 0.0, 1.0)); // 1 rad/s CCW
        cmd.schedule();
        loop(100); // 2 seconds
        cmd.cancel();
        loop(10);

        double turned = drive.getGyroRotation().minus(before).getRadians();
        assertTrue(turned > 0.8, "gyro should have turned about 2 rad CCW, got " + turned);
    }

    /**
     * The X-lock must actually be an X. The old {@code lockWheels()} sent 45, 135, -45, 45 degrees
     * -- the two back wheels were parallel, leaving the robot pushable sideways.
     */
    @Test
    @Order(5)
    void xLockPointsWheelsAtTheChassisCorners() {
        Command cmd = new RunCommand(drive::stopWithX, drive);
        cmd.schedule();
        loop(100);
        cmd.cancel();

        SwerveModuleState[] states = drive.getModuleStates();
        double[] expectedDeg = {45, -45, 135, -135};
        for (int i = 0; i < 4; i++) {
            double actual = states[i].angle.getDegrees();
            double expected = expectedDeg[i];
            // A wheel pointing 180 degrees the other way forms the same X.
            double error =
                    Math.min(
                            Math.abs(Rotation2d.fromDegrees(actual - expected).getDegrees()),
                            Math.abs(Rotation2d.fromDegrees(actual - expected + 180).getDegrees()));
            assertTrue(error < 12.0, "module " + i + " at " + actual + " deg, wanted " + expected);
        }
    }

    // ------------------------------------------------------------------ shooter

    @Test
    @Order(6)
    void flywheelSpinsUpAndReportsAtSpeed() {
        flywheel.start(3000);
        loop(150); // 3 seconds
        assertEquals(3000, flywheel.getLeadVelocityRpm(), 400, "lead flywheel RPM");
        assertEquals(3000, flywheel.getFollowerVelocityRpm(), 400, "follower flywheel RPM");
        assertTrue(flywheel.atSpeed(), "atSpeed should be true once both wheels are there");

        flywheel.stop();
        loop(150);
        assertTrue(
                Math.abs(flywheel.getLeadVelocityRpm()) < 300,
                "flywheel should coast down after stop()");
        assertTrue(!flywheel.atSpeed(), "atSpeed must be false once the wheel has stopped");
    }

    /**
     * atSpeed must be a two-sided band. The old check was {@code |v| > target - tolerance}, so an
     * over-speeding wheel read as ready and a ball left with unintended spin.
     */
    @Test
    @Order(7)
    void atSpeedRejectsOverspeed() {
        flywheel.start(1000);
        loop(200);
        assertTrue(flywheel.atSpeed(), "should reach 1000 rpm");

        // Ask for far less than the wheel is currently doing; it must not report ready while it
        // is still spinning down past the target.
        flywheel.setTargetRpm(200);
        loop(2);
        assertTrue(!flywheel.atSpeed(), "over-speed must not count as at-speed");
        flywheel.stop();
        loop(100);
    }

    @Test
    @Order(8)
    void hoodMovesToTarget() {
        hood.setTargetAngleRad(0.25);
        loop(120);
        assertEquals(0.25, hood.getAngleRad(), 0.03, "hood angle");
        assertTrue(hood.atTarget());

        hood.setTargetAngleRad(ShooterConstants.kHoodLowerRad);
        loop(120);
        assertEquals(ShooterConstants.kHoodLowerRad, hood.getAngleRad(), 0.03);
    }

    /** The hood must refuse commands outside its mechanical travel. */
    @Test
    @Order(9)
    void hoodClampsOutOfRangeCommands() {
        hood.setTargetAngleRad(5.0);
        assertEquals(ShooterConstants.kHoodUpperRad, hood.getTargetAngleRad(), 1e-9);
        hood.setTargetAngleRad(-5.0);
        assertEquals(ShooterConstants.kHoodLowerRad, hood.getTargetAngleRad(), 1e-9);
        loop(120);
    }

    // ------------------------------------------------------------------ intake

    @Test
    @Order(10)
    void pivotDeploysAndStows() {
        pivot.setTargetAngleRad(IntakeConstants.kPivotDeployedRad);
        loop(200); // 4 seconds; the arm sim includes gravity
        assertEquals(IntakeConstants.kPivotDeployedRad, pivot.getAngleRad(), 0.15, "deploy");

        pivot.setTargetAngleRad(IntakeConstants.kPivotStowedRad);
        loop(200);
        assertEquals(IntakeConstants.kPivotStowedRad, pivot.getAngleRad(), 0.15, "stow");
        assertTrue(pivot.atPosition(IntakeConstants.kPivotStowedRad));
    }

    // ------------------------------------------------------------------ demo mode

    /**
     * Demo mode must scale the limits without destroying the commanded values -- an earlier version
     * wrote the demo constant back into the flywheel's target field, so the robot stayed at demo
     * speed after demo mode was turned off.
     */
    @Test
    @Order(11)
    void demoModeScalesButDoesNotClobber() {
        RobotState state = RobotState.getInstance();
        double fullSpeed = drive.getMaxLinearSpeedMetersPerSec();

        flywheel.start(4000);
        state.setDemoMode(true);
        loop(2);
        assertTrue(
                drive.getMaxLinearSpeedMetersPerSec() < fullSpeed,
                "demo mode should reduce the drivetrain limit");
        assertTrue(flywheel.getTargetRpm() < 4000, "demo mode should reduce the shooter velocity");

        state.setDemoMode(false);
        loop(2);
        assertEquals(
                fullSpeed,
                drive.getMaxLinearSpeedMetersPerSec(),
                1e-9,
                "full speed must come back");
        assertEquals(
                4000,
                flywheel.getTargetRpm(),
                1e-9,
                "the commanded velocity must survive demo mode");
        flywheel.stop();
        loop(5);
    }
}
