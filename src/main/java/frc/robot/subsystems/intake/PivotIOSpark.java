package frc.robot.subsystems.intake;

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

/** Spark Max on the pivot, duty-cycle absolute encoder before the chain reduction. */
public class PivotIOSpark implements PivotIO {
    private final SparkMax motor;
    private final RelativeEncoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;
    private final Debouncer motorDebounce = new Debouncer(0.5);

    public PivotIOSpark() {
        motor = new SparkMax(IntakeConstants.kPivotMotorId, MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(IntakeConstants.kPivotEncoderPort);

        var config = new SparkMaxConfig();
        config.inverted(IntakeConstants.kPivotInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.kPivotCurrentLimitAmps)
                .voltageCompensation(12.0);
        config.encoder
                .positionConversionFactor(2 * Math.PI / IntakeConstants.kPivotTotalReduction)
                .velocityConversionFactor(
                        2 * Math.PI / IntakeConstants.kPivotTotalReduction / 60.0);

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
    public void updateInputs(PivotIOInputs inputs) {
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

    /** Absolute encoder rotations to pivot radians. */
    static double decodeAbsolute(double rotations) {
        double angle = rotations * 2 * Math.PI;
        angle *= IntakeConstants.kPivotAbsoluteEncoderInverted ? -1 : 1;
        angle /= IntakeConstants.kPivotChainReduction;
        angle += IntakeConstants.kPivotEncoderOffsetRad;
        // The pivot sweeps about 1.9 rad, so the readable window is placed on (-2*pi, pi] rather
        // than the usual (-pi, pi] -- that is where the stowed position lands with this offset.
        return MathUtil.inputModulus(angle, -2 * Math.PI, Math.PI);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(
                MathUtil.clamp(
                        volts, -IntakeConstants.kPivotMaxVolts, IntakeConstants.kPivotMaxVolts));
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
