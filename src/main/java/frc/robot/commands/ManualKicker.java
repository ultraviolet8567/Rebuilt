package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Kicker;

public class ManualKicker extends Command {
    private final Kicker kicker;

    public ManualKicker(Kicker kicker) {
        this.kicker = kicker;
    }

    @Override
    public void initialize() {
        kicker.start();
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
    }
}
