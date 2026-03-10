package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Mouth;

public class SpinIntake extends Command {
    private Mouth mouth;

    // private Odometry odometry;
    // private Swerve swerve;

    public SpinIntake(Mouth mouth) {
        this.mouth = mouth;
        // this.odometry = odometry;
        // this.swerve = swerve;
        addRequirements(mouth);
    }

    @Override
    public void execute() {
        mouth.start();
    }

    @Override
    public void end(boolean interrupted) {
        mouth.stop();
    }
}
