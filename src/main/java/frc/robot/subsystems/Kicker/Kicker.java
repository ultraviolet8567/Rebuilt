package frc.robot.subsystems.Kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Kicker.KickerIO.KickerIOInputs;

public class Kicker extends SubsystemBase {
	private final KickerIO io;
	private final KickerIOInputs inputs = new KickerIOInputs();
	/*
	* Initialize all components and one-time logic to be completed on boot-up here
	*/
    public Kicker(KickerIO io) {
        this.io = io;
}
@Override
	public void periodic() {
		io.updateInputs(inputs);
		// Logger.processInputs("Shooter", inputs);

		//Logger.recordOutput("Shooter/TargetVelocity", getTargetVelocity());
		//Logger.recordOutput("Shooter/AtVelocity", atVelocity());
	}

    public void kickIn() {
		io.setKickerInputVoltage(KickerConstants.kKickerVoltage.get());
	}


    public void stopKicker() {
		io.stopKicker();
	}
}

