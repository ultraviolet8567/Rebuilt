package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Alerts;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * The autonomous routine picker.
 *
 * <p>The old {@code AutoChooser} had two dropdowns and pasted their values together into a string
 * -- "Shoot" plus "Left" made "Shoot Left" -- then looked that string up in a map. Most of the nine
 * combinations did not name a real routine, and choosing one of those returned {@code null} from
 * {@code getSelectedAuto()}, which {@code Robot.autonomousInit()} quietly skipped. The failure mode
 * was a robot that sat still for fifteen seconds with nothing on the dashboard saying why.
 *
 * <p>One chooser listing the routines that actually exist cannot express an invalid selection.
 * {@link LoggedDashboardChooser} also records the choice into the log, so a post-match replay knows
 * which auto was selected. An {@link Alert} fires if nothing is chosen.
 */
public class AutoSelector {
    private final LoggedDashboardChooser<Command> chooser;
    private final Alert noneSelected =
            Alerts.create("No autonomous routine selected", AlertType.kWarning);

    public AutoSelector() {
        // buildAutoChooser() enumerates the .auto files that are actually deployed, so a routine
        // that was renamed or deleted in PathPlanner disappears from the dashboard instead of
        // becoming a selection that fails at match start.
        chooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
        chooser.addOption("Do Nothing", Commands.none());
    }

    /** Add a non-PathPlanner routine, e.g. a characterisation run. */
    public void addOption(String name, Command command) {
        chooser.addOption(name, command);
    }

    /** Never null: an unselected chooser yields a command that does nothing, visibly. */
    public Command get() {
        Command selected = chooser.get();
        noneSelected.set(selected == null);
        Logger.recordOutput("Auto/Selected", selected == null ? "NONE" : selected.getName());
        return selected == null ? Commands.none().withName("No auto selected") : selected;
    }
}
