package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Kicker;
import org.littletonrobotics.junction.Logger;

public class Kick extends Command {
    private Kicker kicker;
    private boolean kicking;

    public Kick(Kicker kicker) {
        this.kicker = kicker;
        this.kicking = false;

        //   addRequirements();
    }

    @Override
    public void initialize() {
        kicking = true;
        kicker.start();
    }

    @Override
    public void execute() {
        Logger.recordOutput("Shooter/Kicking", kicking);
    }

    @Override
    public void end(boolean interrupted) {
        kicking = false;
        kicker.stop();
    }
}
