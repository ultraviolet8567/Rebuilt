package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.Alerts;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** The intake pivot arm: deploys to collect, stows to travel. */
public class Pivot extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0, 0, 0);
    private final ArmFeedforward ff = new ArmFeedforward(0, 0, 0, 0);

    private final Alert motorDisconnected =
            Alerts.create("Intake pivot motor disconnected", AlertType.kError);
    private final Alert encoderDisconnected =
            Alerts.create("Intake pivot absolute encoder disconnected", AlertType.kWarning);

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismLigament2d pivotLigament;

    private double targetAngleRad = IntakeConstants.kPivotStowedRad;

    public Pivot(PivotIO io) {
        this.io = io;
        applyGains();

        LoggedMechanismRoot2d root = mechanism.getRoot("IntakePivot", 0.7, 0.3);
        pivotLigament =
                root.append(
                        new LoggedMechanismLigament2d(
                                "Intake",
                                IntakeConstants.kPivotArmLengthMeters,
                                90,
                                6,
                                new Color8Bit(Color.kPurple)));

        io.updateInputs(inputs);
        io.seedRelativeEncoder(inputs.absoluteAngleRad);
        targetAngleRad = inputs.absoluteAngleRad;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Pivot", inputs);

        motorDisconnected.set(!inputs.motorConnected);
        encoderDisconnected.set(!inputs.absoluteEncoderConnected);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                this::applyGains,
                IntakeConstants.kPivotP,
                IntakeConstants.kPivotI,
                IntakeConstants.kPivotD,
                IntakeConstants.kPivotS,
                IntakeConstants.kPivotG,
                IntakeConstants.kPivotV,
                IntakeConstants.kPivotA);

        resyncIfPossible();

        double pidVolts = pid.calculate(getAngleRad(), targetAngleRad);
        // ArmFeedforward wants the angle measured from horizontal; kPivotGravityOffsetRad moves
        // the pivot's own zero into that frame.
        double ffVolts = ff.calculate(getAngleRad() + IntakeConstants.kPivotGravityOffsetRad, 0.0);
        io.setVoltage(pidVolts + ffVolts);

        RobotState.getInstance().setIntakeDeployed(!atPosition(IntakeConstants.kPivotStowedRad));

        pivotLigament.setAngle(90 - Units.radiansToDegrees(getAngleRad()));
        Logger.recordOutput("Mechanism2d/IntakePivot", mechanism);
        Logger.recordOutput("Intake/Pivot/TargetAngleRad", targetAngleRad);
        Logger.recordOutput("Intake/Pivot/PidVolts", pidVolts);
        Logger.recordOutput("Intake/Pivot/FfVolts", ffVolts);
    }

    private void resyncIfPossible() {
        if (!inputs.absoluteEncoderConnected) {
            return;
        }
        double diff = Math.abs(inputs.absoluteAngleRad - inputs.relativeAngleRad);
        if (diff > IntakeConstants.kPivotResyncDeadbandRad
                && diff < IntakeConstants.kPivotResyncWindowRad) {
            io.seedRelativeEncoder(inputs.absoluteAngleRad);
        }
    }

    private void applyGains() {
        pid.setPID(
                IntakeConstants.kPivotP.get(),
                IntakeConstants.kPivotI.get(),
                IntakeConstants.kPivotD.get());
        ff.setKs(IntakeConstants.kPivotS.get());
        ff.setKg(IntakeConstants.kPivotG.get());
        ff.setKv(IntakeConstants.kPivotV.get());
        ff.setKa(IntakeConstants.kPivotA.get());
    }

    // ------------------------------------------------------------------ state

    @AutoLogOutput(key = "Intake/Pivot/AngleRad")
    public double getAngleRad() {
        return inputs.relativeAngleRad;
    }

    public boolean atPosition(double angleRad) {
        return Math.abs(angleRad - getAngleRad()) < IntakeConstants.kPivotToleranceRad;
    }

    public boolean atTarget() {
        return atPosition(targetAngleRad);
    }

    // ------------------------------------------------------------------ control

    public void setTargetAngleRad(double angleRad) {
        targetAngleRad =
                MathUtil.clamp(
                        angleRad,
                        IntakeConstants.kPivotStowedRad,
                        IntakeConstants.kPivotDeployedRad);
    }

    // ------------------------------------------------------------------ commands

    /**
     * Drive to an angle and hold it until interrupted.
     *
     * <p>The old {@code SetPivot} set its target in {@code initialize()} and then declared itself
     * finished immediately, so the pivot was only held by whatever the subsystem happened to be
     * doing afterwards; nothing owned the mechanism and a second binding could quietly take over
     * mid-motion. Holding the requirement for as long as the button is held makes ownership match
     * the driver's intent.
     */
    public Command goTo(double angleRad) {
        return run(() -> setTargetAngleRad(angleRad)).withName("Pivot to " + angleRad);
    }

    /** Drive to an angle and finish once it is reached. For autonomous sequences. */
    public Command goToAndWait(double angleRad) {
        return goTo(angleRad).until(this::atTarget).withName("Pivot to " + angleRad + " (wait)");
    }

    /**
     * Rock the arm between stowed and deployed to shake a stuck ball loose.
     *
     * <p>Driven from the FPGA clock rather than a loop counter, so the shake period is a real
     * half-second regardless of loop rate or of how long the command has been queued.
     */
    public Command shake() {
        Timer timer = new Timer();
        return runEnd(
                        () ->
                                setTargetAngleRad(
                                        timer.get() % IntakeConstants.kFeedShakePeriodSecs
                                                        < IntakeConstants.kFeedShakePeriodSecs / 2
                                                ? IntakeConstants.kPivotStowedRad
                                                : IntakeConstants.kPivotDeployedRad),
                        () -> setTargetAngleRad(IntakeConstants.kPivotStowedRad))
                .beforeStarting(timer::restart)
                .withName("Pivot shake");
    }
}
