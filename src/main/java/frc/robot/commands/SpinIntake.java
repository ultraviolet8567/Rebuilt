package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Funnel;

public class SpinIntake extends Command {
    private final Funnel funnel;

    // private Odometry odometry;
    // private Swerve swerve;

    public SpinIntake(Funnel funnel) {
        this.funnel = funnel;
        // this.odometry = odometry;
        // this.swerve = swerve;
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.start();
    }

    @Override
    public void end(boolean interrupted) {
        funnel.stop();
    }
}
