package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.util.SimBattery;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
    private final TalonFX funnelMotor;
    private final TalonFXConfiguration funnelMotorConfig;

    // Motor turns per funnel turn (kFunnelReduction is stored as the inverse).
    private static final double kGearing = 1.0 / IntakeConstants.kFunnelReduction;

    // Desktop simulation only (null on the real robot).
    private final DCMotorSim funnelSim;
    private final TalonFXSimState funnelSimState;

    public Funnel() {
        System.out.println("[Init] Creating Funnel");

        funnelMotor = new TalonFX(CAN.kFunnelPort);
        funnelMotorConfig = new TalonFXConfiguration();
        funnelMotorConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(40))
                        .withStatorCurrentLimitEnable(true));
        funnelMotorConfig.withMotorOutput(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        funnelMotor.getConfigurator().apply(funnelMotorConfig);

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getKrakenX60(1);
            funnelSim =
                    new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                    gearbox, SimConstants.kFunnelMOI, kGearing),
                            gearbox);
            funnelSimState = funnelMotor.getSimState();
            // Match the inversion configured above so sim signs agree with the real motor.
            funnelSimState.Orientation = ChassisReference.Clockwise_Positive;
        } else {
            funnelSim = null;
            funnelSimState = null;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Intake/Funnel/Voltage", funnelMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(
                "Intake/Funnel/VelocityRPS", funnelMotor.getVelocity().getValueAsDouble());
    }

    public void setFunnelVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage, -IntakeConstants.kFunnelVoltage, IntakeConstants.kFunnelVoltage);
        voltage *= IntakeConstants.kFunnelInverted ? -1 : 1;
        funnelMotor.setVoltage(voltage);
    }

    public void start() {
        setFunnelVoltage(IntakeConstants.kFunnelVoltage);
    }

    public void stop() {
        setFunnelVoltage(0);
    }

    // ------------------------------------------------------------------------------------
    // Desktop simulation
    // ------------------------------------------------------------------------------------

    @Override
    public void simulationPeriodic() {
        double dt = Constants.kLoopPeriodSecs;

        funnelSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
        funnelSim.setInputVoltage(funnelSimState.getMotorVoltage());
        funnelSim.update(dt);

        // Phoenix wants ROTOR (motor shaft) position/velocity in rotations and rot/s.
        funnelSimState.setRawRotorPosition(funnelSim.getAngularPositionRotations() * kGearing);
        funnelSimState.setRotorVelocity(funnelSim.getAngularVelocityRPM() / 60.0 * kGearing);

        SimBattery.addCurrent(funnelSim.getCurrentDrawAmps());
    }
}
