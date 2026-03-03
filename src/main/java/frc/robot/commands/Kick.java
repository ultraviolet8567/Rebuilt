package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Kicker;
import org.littletonrobotics.junction.Logger;

public class Kick extends Command {
    private Shooter shooter;

	public Kick(Shooter shooter) {
		this.shooter = shooter;

		// addRequirements(shooter);
	}

    @Override
	public void initialize() {
		shooter.getKicker().start();
	}

    @Override
	public void end(boolean interrupted) {
		shooter.getKicker().stop();
	}
}
