package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Flywheel;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
    private final Flywheel flywheel;

    public Shoot(Flywheel flywheel) {
        this.flywheel = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Shooter/Shooting", true);
        flywheel.start(ShooterConstants.kFlywheelVelocity.get());
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/Shooting", false);
        flywheel.stop();
    }
}
