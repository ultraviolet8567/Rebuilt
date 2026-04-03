package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Flywheel;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Kicker;
import org.littletonrobotics.junction.Logger;

public class RunKicker extends Command {
    private final Flywheel flywheel;
    private final Kicker kicker;

    public RunKicker(Flywheel flywheel, Kicker kicker) {
        this.flywheel = flywheel;
        this.kicker = kicker;

        addRequirements(kicker);
    }

    @Override
    public void execute() {
        if (flywheel.atVelocity(flywheel.getTargetVelocity())) {
            Logger.recordOutput("Shooter/Kicking", true);
            kicker.start();
        } else {
            Logger.recordOutput("Shooter/Kicking", false);
            kicker.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
    }
}
