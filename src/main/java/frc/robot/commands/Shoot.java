package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Kicker;

import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
    private Flywheel flywheel;
    private Kicker kicker;

    public Shoot(Flywheel flywheel,Kicker kicker) {
        this.flywheel = flywheel;
        this.kicker = kicker;

        addRequirements(flywheel,kicker);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Shooter/Shooting", true);
        flywheel.start();
    }

    @Override
    public void execute() {
        if (flywheel.atVelocity(ShooterConstants.kFlywheelMaxVelocity)) {
            kicker.start();
        } else {
            kicker.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/Shooting", false);
        flywheel.stop();
    }
}
