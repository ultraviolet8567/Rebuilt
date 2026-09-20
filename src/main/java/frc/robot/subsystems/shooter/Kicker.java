package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alerts;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The kicker: runs only when the flywheel is up to speed, so a ball is never fed into a slow wheel.
 */
public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
    private final Alert disconnected = Alerts.create("Kicker motor disconnected", AlertType.kError);

    private boolean running = false;

    public Kicker(KickerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Kicker", inputs);
        disconnected.set(!inputs.connected);
        Logger.recordOutput("Shooter/Kicker/Running", running);
    }

    public void start() {
        running = true;
        io.setVoltage(ShooterConstants.kKickerVolts);
    }

    public void stop() {
        running = false;
        io.stop();
    }

    public boolean isRunning() {
        return running;
    }

    // ------------------------------------------------------------------ commands

    /**
     * Default command: feed whenever {@code ready} says the shooter is up to speed.
     *
     * <p>This keeps the interlock in one place instead of leaving each shooting command to remember
     * it.
     */
    public Command feedWhen(BooleanSupplier ready) {
        return runEnd(
                        () -> {
                            if (ready.getAsBoolean()) {
                                start();
                            } else {
                                stop();
                            }
                        },
                        this::stop)
                .withName("Kicker interlock");
    }

    /** Unconditional run, for clearing a jam in the pit. */
    public Command forceRun() {
        return startEnd(this::start, this::stop).withName("Kicker force");
    }
}
