package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alerts;
import org.littletonrobotics.junction.Logger;

/** The funnel rollers. */
public class Funnel extends SubsystemBase {
    private final FunnelIO io;
    private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
    private final Alert disconnected = Alerts.create("Funnel motor disconnected", AlertType.kError);

    public Funnel(FunnelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Funnel", inputs);
        disconnected.set(!inputs.connected);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }

    /**
     * Run the rollers for as long as the command is scheduled.
     *
     * <p>{@code runEnd} rather than {@code startEnd}: the old {@code SpinIntake} wrote the voltage
     * once in {@code initialize()}. A Spark that has not heard from the roboRIO for 100 ms cuts
     * output on its own watchdog, so a long intake could stall silently until the button was
     * released and pressed again. Writing every loop is what keeps the watchdog fed.
     */
    public Command run(boolean reversed) {
        return runEnd(
                        () -> setVoltage(IntakeConstants.kFunnelVolts * (reversed ? -1 : 1)),
                        this::stop)
                .withName(reversed ? "Funnel eject" : "Funnel intake");
    }
}
