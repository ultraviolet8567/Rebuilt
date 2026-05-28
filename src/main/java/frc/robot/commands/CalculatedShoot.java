package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Storage.Indexer;
import org.littletonrobotics.junction.Logger;

public class CalculatedShoot extends Command {
    private final Flywheel flywheel;
    private final Odometry odometry;
    private final Indexer indexer;

    public CalculatedShoot(Flywheel flywheel, Odometry odometry, Indexer indexer) {
        this.flywheel = flywheel;
        this.odometry = odometry;
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Shooter/Shooting", true);
        flywheel.start(flywheel.calculateTargetVelocity(odometry.distToHub()));
        indexer.setIndexerVoltage(StorageConstants.kIndexerVoltage);
        System.out.println(flywheel.calculateTargetVelocity(odometry.distToHub()));
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/Shooting", false);
        flywheel.stop();
        indexer.stop();
    }
}
