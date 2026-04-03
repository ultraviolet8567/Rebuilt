package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter.Flywheel;
import org.littletonrobotics.junction.Logger;

public class CalculatedShoot extends Command {
    private final Flywheel flywheel;
    private final Odometry odometry;

    public CalculatedShoot(Flywheel flywheel, Odometry odometry) {
        this.flywheel = flywheel;
        this.odometry = odometry;
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Shooter/Shooting", true);
        flywheel.start(flywheel.calculateTargetVelocity(odometry.distToHub()));
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/Shooting", false);
        flywheel.stop();
    }
}
