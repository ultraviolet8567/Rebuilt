// SPIN UP COMMAND NOT USED RIGHT NOW

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter.Shooter;
// import org.littletonrobotics.junction.Logger;

public class SpinUp extends Command {
	private Shooter shooter;
	private Odometry odometry;
	private Swerve swerve;

	public SpinUp(Shooter shooter) {
		this.shooter = shooter;
		// this.odometry = odometry;
		// this.swerve = swerve;

		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		
	}

	@Override
	public void execute() {
		
	}

	@Override
	public void end(boolean interrupted) {

	}
}
