package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.SynthesisDevices;

/**
 * The hood as a hinge about the flywheel axis in Autodesk Synthesis.
 *
 * <p>The Sphinx model's hinge reads 0 in the CAD pose, taken to be the hood's lower limit, so the
 * hood angle the robot code uses is {@code kHoodLowerRad + kSign * hinge}. {@code kSign} is the one
 * number to flip if raising the hood in code lowers it in Synthesis.
 */
public class HoodIOSynthesis implements HoodIO {
    private static final double kSign = 1.0;

    private final SynthesisDevices.Motor motor =
            new SynthesisDevices.Motor("Hood", ShooterConstants.kHoodMotorId);
    private final SynthesisDevices.Encoder encoder =
            new SynthesisDevices.Encoder("Hood", ShooterConstants.kHoodMotorId);

    private double volts = 0.0;
    private double encoderOffsetRad = 0.0;

    private double hoodAngle() {
        return ShooterConstants.kHoodLowerRad + kSign * encoder.getPositionRad();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        motor.set(kSign * volts / 12.0);

        double angle = hoodAngle();
        inputs.motorConnected = true;
        inputs.absoluteEncoderConnected = true;
        inputs.relativeAngleRad = angle + encoderOffsetRad;
        inputs.absoluteAngleRad = HoodIOSim.encodeAbsolute(angle);
        inputs.velocityRadPerSec = kSign * encoder.getVelocityRadPerSec();
        inputs.appliedVolts = volts;
    }

    @Override
    public void setVoltage(double v) {
        volts = MathUtil.clamp(v, -ShooterConstants.kHoodMaxVolts, ShooterConstants.kHoodMaxVolts);
    }

    @Override
    public void seedRelativeEncoder(double angleRad) {
        encoderOffsetRad = angleRad - hoodAngle();
    }

    @Override
    public void stop() {
        volts = 0.0;
    }
}
