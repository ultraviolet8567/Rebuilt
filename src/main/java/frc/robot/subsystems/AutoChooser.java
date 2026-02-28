
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.util.VirtualSubsystem;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class AutoChooser extends VirtualSubsystem {
	private static final ShuffleboardTab main = Shuffleboard.getTab("Main");
	private final SendableChooser<String> driveOut;
	private final GenericEntry autoName;

	private final Map<String, PathPlannerAuto> allAutos = new HashMap<String, PathPlannerAuto>();

	public AutoChooser() {
		System.out.println("[Init] Creating AutoChooser");

		driveOut = new SendableChooser<>();
		driveOut.setDefaultOption("None", "");
		driveOut.addOption("Drive Out", "Drive Out");

		// add selectors to shuffleboard
		main.add("Drive Out", driveOut).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1).withPosition(0, 3);
		autoName = main.add("Auto Name", "").withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(2, 0)
				.getEntry();

		for (String pathName : AutoBuilder.getAllAutoNames()) {
			allAutos.put(pathName, new PathPlannerAuto(pathName));
		}

		System.out.println("[Init] Auto routines loaded");
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Auto/Routine", getAutoCommandName());

		autoName.setString(
				allAutos.containsKey(getAutoCommandName()) ? getAutoCommandName() : "Auto routine does not exist");
	}

	// Returns name of pre-defined autonomous command based on Shuffleboard input
	public String getAutoCommandName() {
		String name;
		if (driveOut.getSelected().equals("Drive Out")) {
			name = "Test";
		} else {
            name = "";
        }

        return name;
	}

	public Pose2d getAutoStartingPose() {
		if (getAutoCommandName().equals("Do Nothing")) {
			return new Pose2d();
		} else {
			return allAutos.get(getAutoCommandName()).getStartingPose();
		}
	}

	public Rotation2d getInitialGyroYaw() {
		return getAutoStartingPose().getRotation();
	}

	public PathPlannerAuto getSelectedAuto() {
		String autoCommandName = getAutoCommandName();

		if (autoCommandName.equals("Do Nothing")) {
			return null;
		} else {
			return allAutos.get(autoCommandName);
		}
	}
}
