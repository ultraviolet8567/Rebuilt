package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.Alerts;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** The shooter flywheel: two independently-controlled wheels that must reach a commanded RPM. */
public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private final PIDController leadPid = new PIDController(0, 0, 0);
    private final PIDController followerPid = new PIDController(0, 0, 0);
    private final SimpleMotorFeedforward leadFf = new SimpleMotorFeedforward(0, 0, 0);
    private final SimpleMotorFeedforward followerFf = new SimpleMotorFeedforward(0, 0, 0);

    private final Alert leadDisconnected =
            Alerts.create("Flywheel lead motor disconnected", AlertType.kError);
    private final Alert followerDisconnected =
            Alerts.create("Flywheel follower motor disconnected", AlertType.kError);

    private double targetRpm = ShooterConstants.kFlywheelDefaultRpm;
    private boolean running = false;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        applyGains();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Flywheel", inputs);

        leadDisconnected.set(!inputs.leadConnected);
        followerDisconnected.set(!inputs.followerConnected);

        // Only re-push gains when a tunable actually moved. The previous code wrote all seven
        // gains into the controllers on every one of the 50 loops per second whether or not
        // anything had changed.
        LoggedTunableNumber.ifChanged(
                hashCode(),
                this::applyGains,
                ShooterConstants.kFlywheelP,
                ShooterConstants.kFlywheelI,
                ShooterConstants.kFlywheelD,
                ShooterConstants.kLeadS,
                ShooterConstants.kLeadV,
                ShooterConstants.kLeadA,
                ShooterConstants.kFollowerS,
                ShooterConstants.kFollowerV,
                ShooterConstants.kFollowerA);

        double setpoint = getTargetRpm();
        if (running) {
            // Measurement and command now share a sign because the inversion lives in the motor
            // configuration; the old code had to negate the measurement here.
            io.setVoltage(
                    leadFf.calculate(setpoint)
                            + leadPid.calculate(inputs.leadVelocityRpm, setpoint),
                    followerFf.calculate(setpoint)
                            + followerPid.calculate(inputs.followerVelocityRpm, setpoint));
        } else {
            io.stop();
        }

        RobotState.getInstance().setShooterAtSpeed(running && atSpeed());

        Logger.recordOutput("Shooter/Flywheel/TargetRpm", setpoint);
        Logger.recordOutput("Shooter/Flywheel/Running", running);
        Logger.recordOutput("Shooter/Flywheel/AtSpeed", atSpeed());
    }

    private void applyGains() {
        leadPid.setPID(
                ShooterConstants.kFlywheelP.get(),
                ShooterConstants.kFlywheelI.get(),
                ShooterConstants.kFlywheelD.get());
        followerPid.setPID(
                ShooterConstants.kFlywheelP.get(),
                ShooterConstants.kFlywheelI.get(),
                ShooterConstants.kFlywheelD.get());
        leadFf.setKs(ShooterConstants.kLeadS.get());
        leadFf.setKv(ShooterConstants.kLeadV.get());
        leadFf.setKa(ShooterConstants.kLeadA.get());
        followerFf.setKs(ShooterConstants.kFollowerS.get());
        followerFf.setKv(ShooterConstants.kFollowerV.get());
        followerFf.setKa(ShooterConstants.kFollowerA.get());
    }

    // ------------------------------------------------------------------ state

    /**
     * Commanded velocity, scaled down in demo mode.
     *
     * <p>Demo scaling is applied here, on the way out, rather than by overwriting {@code targetRpm}
     * -- an earlier version assigned the demo constant back into the field inside this getter, so
     * the next loop found the distance-based velocity replaced by the constant and the robot shot
     * at demo speed for the rest of the match.
     */
    @AutoLogOutput(key = "Shooter/Flywheel/CommandedRpm")
    public double getTargetRpm() {
        return RobotState.getInstance().isDemoMode()
                ? targetRpm * ShooterConstants.kDemoScaleFactor
                : targetRpm;
    }

    /**
     * True when <em>both</em> wheels are within tolerance of the setpoint.
     *
     * <p>The old check looked only at the lead wheel, and only checked that it was fast enough --
     * so an over-speeding wheel read as ready. A ball fed into a shooter whose two sides are at
     * different speeds leaves with spin the trajectory did not account for.
     */
    @AutoLogOutput(key = "Shooter/Flywheel/AtSpeedRaw")
    public boolean atSpeed() {
        double setpoint = getTargetRpm();
        return Math.abs(inputs.leadVelocityRpm - setpoint) < ShooterConstants.kFlywheelToleranceRpm
                && Math.abs(inputs.followerVelocityRpm - setpoint)
                        < ShooterConstants.kFlywheelToleranceRpm;
    }

    public double getLeadVelocityRpm() {
        return inputs.leadVelocityRpm;
    }

    public double getFollowerVelocityRpm() {
        return inputs.followerVelocityRpm;
    }

    public boolean isRunning() {
        return running;
    }

    /** Velocity for a shot from this distance, from the range table. */
    public static double rpmForDistance(double distanceMeters) {
        return ShooterConstants.kDistanceToRpm.get(distanceMeters);
    }

    // ------------------------------------------------------------------ control

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    public void start(double rpm) {
        targetRpm = rpm;
        running = true;
    }

    public void stop() {
        running = false;
        io.stop();
    }

    // ------------------------------------------------------------------ commands

    /**
     * Spin up to a fixed velocity for as long as the command runs.
     *
     * <p>Expressed as a factory on the subsystem rather than as a separate {@code Command} class.
     * The old {@code Shoot}, {@code DirectShoot}, {@code SpinIntake}, {@code SpinIndexer}, {@code
     * SetPivot}, {@code SetHood}, {@code ManualKicker} and {@code RunKicker} classes were each a
     * file of boilerplate around one or two lines of behaviour; 37 of the 42 corpus robots build
     * these inline instead.
     */
    public Command runAtRpm(double rpm) {
        return startEnd(() -> start(rpm), this::stop).withName("Flywheel " + (int) rpm);
    }

    /** Spin up to whatever velocity the supplier asks for, re-evaluated every loop. */
    public Command runAtRpm(DoubleSupplier rpm) {
        return runEnd(() -> start(rpm.getAsDouble()), this::stop).withName("Flywheel tracking");
    }
}
