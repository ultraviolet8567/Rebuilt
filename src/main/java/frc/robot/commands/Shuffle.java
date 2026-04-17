package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import org.littletonrobotics.junction.Logger;

public class Shuffle extends Command {
    private final Flywheel flywheel;
    private final Hood hood;

    public Shuffle(Flywheel flywheel, Hood hood) {
        this.flywheel = flywheel;
        this.hood = hood;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Shooter/Shooting", true);
        flywheel.start(ShooterConstants.kFlywheelVelocity.get());
        hood.setTargetPosition(ShooterConstants.kHoodUpper);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/Shooting", false);
        flywheel.stop();
        hood.setTargetPosition(ShooterConstants.kHoodLower);
    }
}
