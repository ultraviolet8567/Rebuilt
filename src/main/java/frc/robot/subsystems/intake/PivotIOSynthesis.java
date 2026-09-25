package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.SynthesisDevices;

/**
 * The intake pivot as a hinge in Autodesk Synthesis.
 *
 * <p>The Sphinx model's hinge reads 0 with the arm stowed, which is how the robot starts a match,
 * and increases as the arm deploys. The pivot angle the robot code uses is therefore {@code
 * kPivotStowedRad + hinge}. Starting stowed matters: the absolute encoder turns 2.5 times per pivot
 * turn, and only the stowed start reads unambiguously. Synthesis turns motor output into a hinge
 * velocity, so volts are sent as a fraction of 12 V and the subsystem's own position loop closes
 * around the hinge angle.
 */
public class PivotIOSynthesis implements PivotIO {
    private final SynthesisDevices.Motor motor =
            new SynthesisDevices.Motor("Pivot", IntakeConstants.kPivotMotorId);
    private final SynthesisDevices.Encoder encoder =
            new SynthesisDevices.Encoder("Pivot", IntakeConstants.kPivotMotorId);

    private double volts = 0.0;
    private double encoderOffsetRad = 0.0;

    private double pivotAngle() {
        return IntakeConstants.kPivotStowedRad + encoder.getPositionRad();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        motor.set(volts / 12.0);

        double angle = pivotAngle();
        inputs.motorConnected = true;
        inputs.absoluteEncoderConnected = true;
        inputs.relativeAngleRad = angle + encoderOffsetRad;
        inputs.absoluteAngleRad = PivotIOSim.encodeAbsolute(angle);
        inputs.velocityRadPerSec = encoder.getVelocityRadPerSec();
        inputs.appliedVolts = volts;
    }

    @Override
    public void setVoltage(double v) {
        volts = MathUtil.clamp(v, -IntakeConstants.kPivotMaxVolts, IntakeConstants.kPivotMaxVolts);
    }

    @Override
    public void seedRelativeEncoder(double angleRad) {
        encoderOffsetRad = angleRad - pivotAngle();
    }

    @Override
    public void stop() {
        volts = 0.0;
    }
}
