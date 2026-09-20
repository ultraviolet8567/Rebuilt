package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Registry of every {@link Alert} the robot can raise, so the whole active set can be logged in one
 * place and summarised for the drive team.
 *
 * <p>The old code reported problems with {@code System.out.println} at construction time, which
 * scrolls off the RioLog before a match starts and never appears on the driver station. An Alert is
 * a persistent, named condition: it shows up on the dashboard for as long as it is true and
 * disappears by itself when it is not. 20 of the 42 corpus robots use Alerts; this manager follows
 * FRC 6002's.
 */
public final class Alerts {
    private Alerts() {}

    private static final List<Alert> instances = new ArrayList<>();

    /** Create and register an alert. It starts inactive; call {@code set(true)} to raise it. */
    public static Alert create(String text, AlertType type) {
        Alert alert = new Alert(text, type);
        instances.add(alert);
        return alert;
    }

    /** True if any registered alert of this type is currently active. */
    public static boolean hasType(AlertType type) {
        return instances.stream().filter(Alert::get).anyMatch(a -> a.getType() == type);
    }

    /** Publish the currently-active alerts to the log. Call once per loop. */
    public static void log() {
        Logger.recordOutput(
                "Alerts/Active",
                instances.stream()
                        .filter(Alert::get)
                        .map(a -> prefix(a.getType()) + a.getText())
                        .toArray(String[]::new));
        Logger.recordOutput("Alerts/HasError", hasType(AlertType.kError));
        Logger.recordOutput("Alerts/HasWarning", hasType(AlertType.kWarning));
    }

    private static String prefix(AlertType type) {
        return switch (type) {
            case kError -> "Error: ";
            case kWarning -> "Warning: ";
            case kInfo -> "Info: ";
        };
    }
}
