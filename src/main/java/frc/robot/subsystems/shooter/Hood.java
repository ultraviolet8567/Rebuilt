package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alerts;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * The hood: sets the launch angle, positioned against a relative encoder that the absolute one
 * re-seeds.
 */
public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0, 0, 0);

    private final Alert motorDisconnected =
            Alerts.create("Hood motor disconnected", AlertType.kError);
    private final Alert encoderDisconnected =
            Alerts.create("Hood absolute encoder disconnected", AlertType.kWarning);

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismLigament2d hoodLigament;

    private double targetAngleRad = ShooterConstants.kHoodLowerRad;

    public Hood(HoodIO io) {
        this.io = io;
        applyGains();

        LoggedMechanismRoot2d root = mechanism.getRoot("Shooter", 0.5, 0.2);
        root.append(
                new LoggedMechanismLigament2d("Barrel", 0.3, 70, 8, new Color8Bit(Color.kGray)));
        hoodLigament =
                root.append(
                        new LoggedMechanismLigament2d(
                                "Hood", 0.25, 90, 6, new Color8Bit(Color.kOrange)));

        // Seed from the absolute encoder once at boot so the hood knows where it is before it is
        // asked to move. Without this the relative encoder reads zero wherever the hood actually
        // is, and the first position command slams it into a hard stop.
        io.updateInputs(inputs);
        io.seedRelativeEncoder(inputs.absoluteAngleRad);
        targetAngleRad = inputs.absoluteAngleRad;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Hood", inputs);

        motorDisconnected.set(!inputs.motorConnected);
        encoderDisconnected.set(!inputs.absoluteEncoderConnected);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                this::applyGains,
                ShooterConstants.kHoodP,
                ShooterConstants.kHoodI,
                ShooterConstants.kHoodD);

        resyncIfPossible();

        io.setVoltage(pid.calculate(getAngleRad(), targetAngleRad));

        hoodLigament.setAngle(90 + Units.radiansToDegrees(getAngleRad()));
        Logger.recordOutput("Mechanism2d/Hood", mechanism);
        Logger.recordOutput("Shooter/Hood/TargetAngleRad", targetAngleRad);
        Logger.recordOutput("Shooter/Hood/AtTarget", atTarget());
    }

    /**
     * Re-seed the relative encoder from the absolute one when the two are close enough that the
     * absolute reading is trustworthy.
     *
     * <p>The absolute encoder is geared before the rack, so it only reads unambiguously over a
     * window of about {@code 2*pi/kHoodRackReduction} radians of hood travel; outside that it
     * aliases to a value that looks plausible but is wrong. Re-seeding only within the window --
     * and only when the disagreement is larger than encoder noise -- corrects relative-encoder
     * drift without ever acting on an aliased reading.
     */
    private void resyncIfPossible() {
        if (!inputs.absoluteEncoderConnected) {
            return;
        }
        double diff = Math.abs(inputs.absoluteAngleRad - inputs.relativeAngleRad);
        if (diff > ShooterConstants.kHoodResyncDeadbandRad
                && diff < ShooterConstants.kHoodResyncWindowRad) {
            io.seedRelativeEncoder(inputs.absoluteAngleRad);
        }
    }

    private void applyGains() {
        pid.setPID(
                ShooterConstants.kHoodP.get(),
                ShooterConstants.kHoodI.get(),
                ShooterConstants.kHoodD.get());
    }

    // ------------------------------------------------------------------ state

    @AutoLogOutput(key = "Shooter/Hood/AngleRad")
    public double getAngleRad() {
        return inputs.relativeAngleRad;
    }

    public double getTargetAngleRad() {
        return targetAngleRad;
    }

    public boolean atTarget() {
        return Math.abs(getAngleRad() - targetAngleRad) < ShooterConstants.kHoodToleranceRad;
    }

    /**
     * Hood angle for a shot from this distance.
     *
     * <p>Uses {@link ShooterConstants#kDistanceToHoodAngle} once it has been populated from range
     * testing. Until then it returns the long-shot position the robot uses today, so behaviour on
     * the field is unchanged and no invented calibration is in the control path.
     */
    public static double angleForDistance(double distanceMeters) {
        if (ShooterConstants.kDistanceToHoodAngle.get(distanceMeters) == null) {
            return ShooterConstants.kHoodUpperRad;
        }
        return ShooterConstants.kDistanceToHoodAngle.get(distanceMeters);
    }

    // ------------------------------------------------------------------ control

    /** Command a hood angle. Always clamped into the mechanical range. */
    public void setTargetAngleRad(double angleRad) {
        targetAngleRad =
                MathUtil.clamp(
                        angleRad, ShooterConstants.kHoodLowerRad, ShooterConstants.kHoodUpperRad);
    }

    // ------------------------------------------------------------------ commands

    /** Hold a fixed angle while the command runs, returning to the stowed angle afterwards. */
    public Command goTo(double angleRad) {
        return startEnd(
                        () -> setTargetAngleRad(angleRad),
                        () -> setTargetAngleRad(ShooterConstants.kHoodLowerRad))
                .withName("Hood to " + angleRad);
    }

    /** Track a moving target angle, re-evaluated every loop. */
    public Command goTo(java.util.function.DoubleSupplier angleRad) {
        return runEnd(
                        () -> setTargetAngleRad(angleRad.getAsDouble()),
                        () -> setTargetAngleRad(ShooterConstants.kHoodLowerRad))
                .withName("Hood tracking");
    }

    /** Operator trim: walk the target up or down while the button is held. */
    public Command trim(boolean down) {
        return run(() ->
                        setTargetAngleRad(
                                targetAngleRad
                                        + (down ? -1 : 1) * ShooterConstants.kHoodTrimStepRad))
                .withName("Hood trim " + (down ? "down" : "up"));
    }
}
