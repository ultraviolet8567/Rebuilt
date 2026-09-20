package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake.Pivot;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Swerve;
import frc.robot.util.SimBattery;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * Boots the whole robot in desktop simulation (no hardware, no Driver Station app) and drives the
 * simulated mechanisms to make sure the physics models, the simulated sensors and the team's
 * control code all agree with each other. Run with: ./gradlew test
 *
 * <p>The tests share one RobotContainer because motor controllers can only be created once per CAN
 * ID per process, so they run in a fixed order.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class SimulationSmokeTest {
    private static RobotContainer container;
    private static Swerve swerve;
    private static Odometry odometry;
    private static Flywheel flywheel;
    private static Hood hood;
    private static Pivot pivot;

    @BeforeAll
    static void bootRobot() {
        assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");

        // Pretend a Driver Station is attached and the robot is enabled in teleop.
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();

        // AdvantageKit is deliberately NOT started here (it refuses to start outside a
        // LoggedRobot). Every Logger.recordOutput() call is guarded by an "is running" check,
        // so they are cheap no-ops in this test.

        // Same construction path as Robot(): this also proves the robot boots with NO alliance
        // assigned, which used to crash in Odometry.resetHeading().
        container = new RobotContainer();
        assertNotNull(container);

        swerve = container.getSwerve();
        odometry = container.getOdometry();
        flywheel = container.getShooter().getFlywheel();
        hood = container.getShooter().getHood();
        pivot = container.getIntake().getPivot();
    }

    @AfterAll
    static void shutdown() {
        CommandScheduler.getInstance().cancelAll();
        HAL.shutdown();
    }

    /** One 20 ms robot loop: scheduler (periodic + simulationPeriodic) then the battery model. */
    private static void loop(int cycles) {
        for (int i = 0; i < cycles; i++) {
            DriverStationSim.notifyNewData();
            DriverStation.refreshData();
            CommandScheduler.getInstance().run();
            SimBattery.update();
            // Give the Phoenix 6 (Pigeon) simulation thread a little real time to publish.
            Timer.delay(0.002);
        }
    }

    /** Run a command that owns the swerve so the joystick default command stays out of the way. */
    private static Command driveCommand(ChassisSpeeds speeds) {
        return new RunCommand(() -> swerve.setModuleStates(speeds), swerve);
    }

    @Test
    @Order(1)
    void encodersAgreeAtBoot() {
        loop(5);
        assertEquals(hood.getAbsoluteRotationRads(), hood.getRelativeRotationRads(), 0.01, "hood");
        assertEquals(
                pivot.getAbsoluteRotationRads(), pivot.getRelativeRotationRads(), 0.01, "pivot");
        assertEquals(IntakeConstants.kPivotLower, pivot.getRelativeRotationRads(), 0.02);
        assertEquals(ShooterConstants.kHoodLower, hood.getRelativeRotationRads(), 0.02);
    }

    @Test
    @Order(2)
    void drivingForwardMovesTheRobot() {
        Command cmd = driveCommand(new ChassisSpeeds(1.0, 0.0, 0.0));
        cmd.schedule();
        loop(100); // 2 seconds
        cmd.cancel();

        SwerveModuleState[] states = swerve.getModuleStates();
        for (SwerveModuleState s : states) {
            assertTrue(
                    s.speedMetersPerSecond > 0.5,
                    "module should be moving forward, got " + s.speedMetersPerSecond);
            assertEquals(0.0, s.angle.getRadians(), 0.15, "wheels should point straight ahead");
        }

        // Odometry (built from the simulated encoders) and the sim ground truth should both
        // show the robot having moved roughly 1 m/s * 2 s, allowing for spin-up.
        double odoX = odometry.getPose().getX();
        double trueX = swerve.getSimTruePose().getX();
        assertTrue(odoX > 1.0 && odoX < 2.5, "odometry x = " + odoX);
        assertEquals(trueX, odoX, 0.2, "odometry should track sim ground truth");
        assertEquals(0.0, odometry.getPose().getY(), 0.2, "no sideways drift");
    }

    @Test
    @Order(3)
    void modulesCanSteer() {
        Command cmd = driveCommand(new ChassisSpeeds(0.0, 1.0, 0.0)); // strafe left = wheels at 90
        cmd.schedule();
        loop(75);
        cmd.cancel();

        for (SwerveModuleState s : swerve.getModuleStates()) {
            double angle = Math.abs(s.angle.getRadians());
            assertEquals(Math.PI / 2, angle, 0.15, "wheel should have steered to +/-90 deg");
        }
    }

    @Test
    @Order(4)
    void rotatingUpdatesTheSimulatedGyro() {
        Rotation2d before = odometry.getGyrometerHeading();
        Command cmd = driveCommand(new ChassisSpeeds(0.0, 0.0, 1.0)); // 1 rad/s CCW
        cmd.schedule();
        loop(100); // 2 seconds
        cmd.cancel();
        loop(10);

        double turned = odometry.getGyrometerHeading().minus(before).getRadians();
        assertTrue(turned > 0.8, "gyro should have turned about 2 rad CCW, got " + turned);
    }

    @Test
    @Order(5)
    void flywheelSpinsUpAndReportsAtVelocity() {
        flywheel.start(3000);
        loop(150); // 3 seconds
        double rpm = -flywheel.getVelocity(flywheel.getLeadMotor());
        // Control is feedforward-dominated (kV ~ free speed), so allow a generous band.
        assertEquals(3000, rpm, 400, "lead flywheel RPM");
        assertTrue(flywheel.atVelocity(3000), "atVelocity should be true");
        flywheel.stop();
        loop(150);
        assertTrue(
                Math.abs(flywheel.getVelocity(flywheel.getLeadMotor())) < 300,
                "flywheel should coast down after stop()");
    }

    @Test
    @Order(6)
    void hoodMovesToTarget() {
        hood.setTargetPosition(0.25);
        loop(100);
        assertEquals(0.25, hood.getRelativeRotationRads(), 0.03, "hood relative");
        // 0.25 rad is inside the absolute encoder's readable window, so it agrees too.
        assertEquals(0.25, hood.getAbsoluteRotationRads(), 0.03, "hood absolute");
        hood.setTargetPosition(ShooterConstants.kHoodLower);
        loop(100);
        assertEquals(ShooterConstants.kHoodLower, hood.getRelativeRotationRads(), 0.03);
    }

    @Test
    @Order(7)
    void pivotDeploysAndStows() {
        pivot.setTargetPosition(IntakeConstants.kPivotUpper);
        loop(200); // 4 seconds; the arm sim includes gravity
        assertEquals(IntakeConstants.kPivotUpper, pivot.getRelativeRotationRads(), 0.15, "deploy");
        pivot.setTargetPosition(IntakeConstants.kPivotLower);
        loop(200);
        assertEquals(IntakeConstants.kPivotLower, pivot.getRelativeRotationRads(), 0.15, "stow");
        assertTrue(pivot.atPosition(IntakeConstants.kPivotLower));
    }
}
