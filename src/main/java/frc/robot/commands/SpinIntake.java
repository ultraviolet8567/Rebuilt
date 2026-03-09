package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Raiser;
import frc.robot.subsystems.Intake.Spinner;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve;
public class SpinIntake extends Command {
    private Spinner spinner;
    private Odometry odometry;
    private Swerve swerve;

    public SpinIntake(Odometry odometry, Spinner spinner, Swerve swerve) {
        this.spinner = spinner;
        this.odometry = odometry;
        this.swerve = swerve;
    addRequirements(spinner);
    }
    @Override
    public void execute() {
        spinner.spinStart();
    }

    @Override
    public void end(boolean interrupted) {
        spinner.spinStop();
    }

}
