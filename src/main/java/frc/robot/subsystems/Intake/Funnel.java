package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class Funnel extends SubsystemBase {

    // Idk what variables 🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🍅

    private final TalonFX funnelMotor; // Kraken Spins

    public Funnel() {
        System.out.println("[Init] Creating Funnel");

        funnelMotor = new TalonFX(CAN.kFunnelPort);
    }

    // Intake Constructer Close Bracket

    // public void setRaiseVelocity()

    public void setFunnelVoltage(double voltage) {

        voltage =
                MathUtil.clamp(
                        voltage,
                        -IntakeConstants.kFunnelMaxVoltage,
                        IntakeConstants.kFunnelMaxVoltage);
        voltage *= IntakeConstants.kFunnelInverted ? 1 : -1;
        funnelMotor.setVoltage(voltage);
    }

    // help pls 💔

    public void start() {
        setFunnelVoltage(IntakeConstants.kFunnelVoltage);
    }

    public void stop() {
        setFunnelVoltage(0);
    }
} // DO NOT CODE PAST THIS LINE
