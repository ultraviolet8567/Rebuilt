package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.Hood;

public class DirectHood extends Command {
    private final Hood hood;

    public DirectHood(Hood hood) {
        this.hood = hood;

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setHoodVoltage(-ShooterConstants.kHoodTestVoltage.get());
    }

    @Override
    public void end(boolean interrupted) {
        hood.stop();
    }
}
