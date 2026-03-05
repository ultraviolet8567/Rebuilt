package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Hood;

public class HoodDown extends Command {
    private Hood hood;

    public HoodDown(Hood hood) {
        this.hood = hood;

        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setAngleRads(-Math.PI);
    }

    @Override
    public void end(boolean interrupted) {
        hood.stop();
    }
}
