package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.function.DoubleSupplier;

/** Spark Max driving the hood, with a duty-cycle absolute encoder on the rack shaft. */
public class HoodIOSpark implements HoodIO {
    private final SparkMax motor;
    private final RelativeEncoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;
    private final Debouncer motorDebounce = new Debouncer(0.5);

    public HoodIOSpark() {
        motor = new SparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(ShooterConstants.kHoodEncoderPort);

        var config = new SparkMaxConfig();
        config.inverted(ShooterConstants.kHoodInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ShooterConstants.kHoodCurrentLimitAmps)
                .voltageCompensation(12.0);
        // Report the hood angle directly in radians rather than in motor rotations, so nothing
        // downstream has to remember the reduction or which way round the encoder counts.
        config.encoder
                .positionConversionFactor(2 * Math.PI / ShooterConstants.kHoodTotalReduction)
                .velocityConversionFactor(
                        2 * Math.PI / ShooterConstants.kHoodTotalReduction / 60.0);

        tryUntilOk(
                motor,
                5,
                () ->
                        motor.configure(
                                config,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(motor, relativeEncoder::getPosition, v -> inputs.relativeAngleRad = v);
        ifOk(motor, relativeEncoder::getVelocity, v -> inputs.velocityRadPerSec = v);
        ifOk(
                motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                v -> inputs.appliedVolts = v[0] * v[1]);
        ifOk(motor, motor::getOutputCurrent, v -> inputs.currentAmps = v);
        inputs.motorConnected = motorDebounce.calculate(!sparkStickyFault);

        inputs.absoluteEncoderConnected = absoluteEncoder.isConnected();
        inputs.absoluteAngleRad = decodeAbsolute(absoluteEncoder.get());
    }

    /** Absolute encoder rotations to hood radians. Inverse of the seeding maths in simulation. */
    static double decodeAbsolute(double rotations) {
        double angle = rotations * 2 * Math.PI;
        angle *= ShooterConstants.kHoodAbsoluteEncoderInverted ? -1 : 1;
        angle /= ShooterConstants.kHoodRackReduction;
        angle += ShooterConstants.kHoodEncoderOffsetRad;
        return MathUtil.inputModulus(angle, -Math.PI, Math.PI);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(
                MathUtil.clamp(
                        volts, -ShooterConstants.kHoodMaxVolts, ShooterConstants.kHoodMaxVolts));
    }

    @Override
    public void seedRelativeEncoder(double angleRad) {
        tryUntilOk(motor, 5, () -> relativeEncoder.setPosition(angleRad));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
