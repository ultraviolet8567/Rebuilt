// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Alerts;
import frc.robot.util.SimBattery;
import java.io.File;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Mode transitions and the loop. Everything else lives in {@link RobotContainer}. */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    private final Alert brownout =
            Alerts.create("Brownout: battery voltage collapsed", AlertType.kError);
    private final Alert lowBattery =
            Alerts.create("Battery below 11.5 V -- swap before the next match", AlertType.kWarning);
    private final Alert canError = Alerts.create("CAN bus errors detected", AlertType.kWarning);

    private static final double kCoastDelaySecs = 3.0;
    private final Timer disabledTimer = new Timer();
    private boolean brakeModeApplied = true;

    public Robot() {
        // AdvantageKit must be configured and started BEFORE any subsystem is created, or the
        // first loop of inputs is logged into the void.
        Logger.recordMetadata("ProjectName", "Rebuilt2026");
        Logger.recordMetadata("RuntimeMode", Constants.currentMode.toString());
        Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));

        switch (Constants.currentMode) {
            case REAL -> {
                // roboRIO internal flash. Clean out /home/lvuser/logs periodically.
                Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs/"));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                new File("logs").mkdirs();
                Logger.addDataReceiver(new WPILOGWriter("logs/"));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                // Feed a recorded log back through this code as fast as the CPU allows, writing
                // the result out beside it for comparison. This is what the IO layer buys: a
                // control-code change can be tested against a real match before the robot is
                // powered on.
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
            }
        }

        Logger.start();

        // Joystick ports 2-5 are unused; without this the driver station fills the log with
        // warnings that hide the ones that matter.
        DriverStation.silenceJoystickConnectionWarning(true);

        robotContainer = new RobotContainer();

        if (Constants.tuningMode) {
            Alerts.create("Tuning mode is ON -- turn it off for competition", AlertType.kInfo)
                    .set(true);
        }
    }

    @Override
    public void robotPeriodic() {
        // Run the scheduler at real-time priority. Every other thread on the roboRIO -- the web
        // server, the CAN receive threads, garbage collection -- can otherwise delay the control
        // loop, and a control loop that runs late is a control loop with the wrong dt.
        Threads.setCurrentThreadPriority(true, 99);

        CommandScheduler.getInstance().run();

        robotContainer.updateDashboardInputs();
        RobotState.getInstance().update();

        double voltage = RobotController.getBatteryVoltage();
        brownout.set(RobotController.isBrownedOut());
        lowBattery.set(voltage < 11.5 && DriverStation.isDisabled());
        canError.set(RobotController.getCANStatus().receiveErrorCount > 0);
        Logger.recordOutput("Robot/BatteryVoltage", voltage);
        Logger.recordOutput(
                "Robot/CanUtilization", RobotController.getCANStatus().percentBusUtilization);
        Alerts.log();

        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledInit() {
        RobotState.getInstance().setMode(RobotState.Mode.DISABLED);
        robotContainer.setDisabledMode(true);
        disabledTimer.restart();
        brakeModeApplied = true;
    }

    /**
     * Coast the drive motors a few seconds after the robot is disabled, so it can be pushed off the
     * field -- but not immediately, because a robot disabled at speed would keep rolling into
     * whatever is in front of it.
     */
    @Override
    public void disabledPeriodic() {
        if (brakeModeApplied && disabledTimer.hasElapsed(kCoastDelaySecs)) {
            robotContainer.getDrive().setBrakeMode(false);
            brakeModeApplied = false;
        }
    }

    private void applyBrakeMode() {
        if (!brakeModeApplied) {
            robotContainer.getDrive().setBrakeMode(true);
            brakeModeApplied = true;
        }
        disabledTimer.stop();
    }

    @Override
    public void autonomousInit() {
        RobotState.getInstance().setMode(RobotState.Mode.AUTO);
        robotContainer.setDisabledMode(false);
        applyBrakeMode();

        autonomousCommand = robotContainer.getAutonomousCommand();
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    @Override
    public void teleopInit() {
        RobotState.getInstance().setMode(RobotState.Mode.TELEOP);
        robotContainer.setDisabledMode(false);
        applyBrakeMode();

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        RobotState.getInstance().setMode(RobotState.Mode.TEST);
        robotContainer.setDisabledMode(false);
        applyBrakeMode();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationInit() {
        System.out.println("[Init] Running in DESKTOP SIMULATION. No hardware is being driven.");
    }

    /**
     * Turn the current every simulated mechanism reported this loop into a loaded battery voltage.
     * Each {@code *IOSim} adds its draw during {@code updateInputs}, which the scheduler has
     * already called by the time this runs.
     */
    @Override
    public void simulationPeriodic() {
        SimBattery.update();
    }
}
