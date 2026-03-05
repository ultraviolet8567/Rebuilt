package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Flywheel;

public class DirectShoot extends Command {
    private Flywheel flywheel;

    public DirectShoot(Flywheel flywheel) {
        this.flywheel = flywheel;

        // addRequirements(shooter);
    }

    @Override
    public void initialize() {
        flywheel.setVoltage(ShooterConstants.kFlywheelVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}
