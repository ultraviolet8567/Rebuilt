package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.Funnel;

public class SpinIntake extends Command {
    private final Funnel funnel;
    private final boolean reversed;

    // private Odometry odometry;
    // private Swerve swerve;

    public SpinIntake(Funnel funnel, boolean reversed) {
        this.funnel = funnel;
        this.reversed = reversed;
        // this.odometry = odometry;
        // this.swerve = swerve;
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.setFunnelVoltage(IntakeConstants.kFunnelVoltage * (reversed ? -1 : 1));
    }

    @Override
    public void end(boolean interrupted) {
        funnel.stop();
    }
}
