package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Flywheel;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
    private Flywheel flywheel;
    private boolean shooting;

    public Shoot(Flywheel flywheel) {
        this.flywheel = flywheel;
        this.shooting = false;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        shooting = true;
        flywheel.start();
    }

    @Override
    public void execute() {
        Logger.recordOutput("Shooter/Shooting", shooting);
    }

    @Override
    public void end(boolean interrupted) {
        shooting = false;
        flywheel.stop();
    }
}
