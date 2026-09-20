package frc.robot.subsystems.storage;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/** Spark Flex driving the indexer. */
public class IndexerIOSpark implements IndexerIO {
    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final Debouncer debounce = new Debouncer(0.5);

    public IndexerIOSpark() {
        motor = new SparkFlex(StorageConstants.kIndexerMotorId, MotorType.kBrushless);
        encoder = motor.getEncoder();

        var config = new SparkFlexConfig();
        config.inverted(StorageConstants.kIndexerInverted)
                .smartCurrentLimit(StorageConstants.kIndexerCurrentLimitAmps)
                .voltageCompensation(12.0);
        config.encoder.velocityConversionFactor(1.0 / StorageConstants.kIndexerReduction);

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
    public void updateInputs(IndexerIOInputs inputs) {
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
                        volts, -StorageConstants.kIndexerVolts, StorageConstants.kIndexerVolts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
