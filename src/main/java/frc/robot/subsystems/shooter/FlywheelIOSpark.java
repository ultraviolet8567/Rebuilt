package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/** Two Spark Flex controllers, one per side of the shooter. */
public class FlywheelIOSpark implements FlywheelIO {
    private final SparkFlex leadMotor;
    private final SparkFlex followerMotor;
    private final RelativeEncoder leadEncoder;
    private final RelativeEncoder followerEncoder;

    private final Debouncer leadDebounce = new Debouncer(0.5);
    private final Debouncer followerDebounce = new Debouncer(0.5);

    public FlywheelIOSpark() {
        leadMotor = new SparkFlex(ShooterConstants.kFlywheelLeadId, MotorType.kBrushless);
        followerMotor = new SparkFlex(ShooterConstants.kFlywheelFollowerId, MotorType.kBrushless);
        leadEncoder = leadMotor.getEncoder();
        followerEncoder = followerMotor.getEncoder();

        var config = new SparkFlexConfig();
        // Inversion belongs here, not at the call site: this flips the encoder with the command,
        // so a positive command reads back as a positive velocity.
        config.inverted(ShooterConstants.kFlywheelInverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(ShooterConstants.kFlywheelCurrentLimitAmps)
                .voltageCompensation(12.0);
        config.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelReduction);

        tryUntilOk(
                leadMotor,
                5,
                () ->
                        leadMotor.configure(
                                config,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));
        tryUntilOk(
                followerMotor,
                5,
                () ->
                        followerMotor.configure(
                                config,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(leadMotor, leadEncoder::getVelocity, v -> inputs.leadVelocityRpm = v);
        ifOk(
                leadMotor,
                new DoubleSupplier[] {leadMotor::getAppliedOutput, leadMotor::getBusVoltage},
                v -> inputs.leadAppliedVolts = v[0] * v[1]);
        ifOk(leadMotor, leadMotor::getOutputCurrent, v -> inputs.leadCurrentAmps = v);
        inputs.leadConnected = leadDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(followerMotor, followerEncoder::getVelocity, v -> inputs.followerVelocityRpm = v);
        ifOk(
                followerMotor,
                new DoubleSupplier[] {
                    followerMotor::getAppliedOutput, followerMotor::getBusVoltage
                },
                v -> inputs.followerAppliedVolts = v[0] * v[1]);
        ifOk(followerMotor, followerMotor::getOutputCurrent, v -> inputs.followerCurrentAmps = v);
        inputs.followerConnected = followerDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setVoltage(double leadVolts, double followerVolts) {
        leadMotor.setVoltage(clamp(leadVolts));
        followerMotor.setVoltage(clamp(followerVolts));
    }

    private static double clamp(double volts) {
        return MathUtil.clamp(
                volts, -ShooterConstants.kFlywheelMaxVolts, ShooterConstants.kFlywheelMaxVolts);
    }

    @Override
    public void stop() {
        leadMotor.stopMotor();
        followerMotor.stopMotor();
    }
}
