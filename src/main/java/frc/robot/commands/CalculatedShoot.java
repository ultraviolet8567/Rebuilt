package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Odometry;
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
        flywheel.start(flywheel.calculateTargetVelocity(odometry.getDistToRobot()));
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/Shooting", false);
        flywheel.stop();
    }

}

