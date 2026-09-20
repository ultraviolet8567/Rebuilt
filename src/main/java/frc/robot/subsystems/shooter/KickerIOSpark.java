package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/** Spark Max driving the kicker wheel. */
public class KickerIOSpark implements KickerIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final Debouncer debounce = new Debouncer(0.5);

    public KickerIOSpark() {
        motor = new SparkMax(ShooterConstants.kKickerId, MotorType.kBrushless);
        encoder = motor.getEncoder();

        var config = new SparkMaxConfig();
        config.inverted(ShooterConstants.kKickerInverted)
                .smartCurrentLimit(ShooterConstants.kKickerCurrentLimitAmps)
                .voltageCompensation(12.0);
        config.encoder.velocityConversionFactor(1.0 / ShooterConstants.kKickerReduction);

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
    public void updateInputs(KickerIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(motor, encoder::getVelocity, v -> inputs.velocityRpm = v);
        ifOk(
                motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                v -> inputs.appliedVolts = v[0] * v[1]);
        ifOk(motor, motor::getOutputCurrent, v -> inputs.currentAmps = v);
        inputs.connected = debounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(
                MathUtil.clamp(
                        volts, -ShooterConstants.kKickerVolts, ShooterConstants.kKickerVolts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
