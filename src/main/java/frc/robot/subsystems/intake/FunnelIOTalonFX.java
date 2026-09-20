package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Kraken X60 on the funnel rollers. */
public class FunnelIOTalonFX implements FunnelIO {
    private final TalonFX motor = new TalonFX(IntakeConstants.kFunnelMotorId);
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> current = motor.getStatorCurrent();

    public FunnelIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(IntakeConstants.kFunnelCurrentLimitAmps))
                        .withStatorCurrentLimitEnable(true));
        config.withMotorOutput(
                new MotorOutputConfigs()
                        .withInverted(
                                IntakeConstants.kFunnelInverted
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));
        motor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(velocity, appliedVolts, current).isOK();
        inputs.velocityRps = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(
                MathUtil.clamp(volts, -IntakeConstants.kFunnelVolts, IntakeConstants.kFunnelVolts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
