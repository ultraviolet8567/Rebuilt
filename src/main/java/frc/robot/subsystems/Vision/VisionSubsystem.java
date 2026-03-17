package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    private List<Pair<Vision, VisionInputsAutoLogged>> inputsList = new ArrayList<>();

    public VisionSubsystem(VisionIO... visionIOList) {
        for (VisionIO visionIO : visionIOList) {
            inputsList.add(Pair.of(visionIO, new VisionIOInputsAutoLogged()));
        }
    }

    @Override
    public void periodic() {
        List<RobotState.AprilTagObservation> aprilTagObservations = new ArrayList<>();
        for (Pair<VisionIO, VisionIOInputsAutoLogged> camera : inputsList) {
            var io = camera.getFirst();
            var inputs = camera.getSecond();
            io.updateInputs(inputs);
            Logger.processInputs(io.getLimelightName(), inputs);
            if (inputs.hasTargets && inputs.robotPoseBasedOffDistanceCalcAndTagLocation != null) {
                aprilTagObservations.add(new RobotState.AprilTagObservation(
                        io.getLimelightName(),
                        io.getLimelightLocation(),
                        inputs.tagId,
                        inputs.robotPoseBasedOffDistanceCalcAndTagLocation));
            }
        }

        RobotState.getInstance()
                .addVisionObservation(aprilTagObservations.toArray(new RobotState.AprilTagObservation[0]));
    }

    public void setThrottleValue(int throttleValue) {
        for (Pair<VisionIO, VisionIOInputsAutoLogged> camera : inputsList) {
            var io = camera.getFirst();
            io.setThrottleValue(throttleValue);
        }
    }
}
