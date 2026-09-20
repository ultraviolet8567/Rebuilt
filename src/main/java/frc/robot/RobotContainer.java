// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoSelector;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Funnel;
import frc.robot.subsystems.intake.FunnelIO;
import frc.robot.subsystems.intake.FunnelIOSim;
import frc.robot.subsystems.intake.FunnelIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.PivotIOSpark;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOSpark;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.HoodIOSim;
import frc.robot.subsystems.shooter.HoodIOSpark;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.KickerIO;
import frc.robot.subsystems.shooter.KickerIOSim;
import frc.robot.subsystems.shooter.KickerIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.storage.Indexer;
import frc.robot.subsystems.storage.IndexerIO;
import frc.robot.subsystems.storage.IndexerIOSim;
import frc.robot.subsystems.storage.IndexerIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;

/**
 * Builds the robot: picks a hardware implementation for each subsystem based on where the code is
 * running, wires the controllers, and hands the autonomous routine to {@link Robot}.
 *
 * <p>The mode switch in the constructor is the whole point of the IO layer. Nothing below this
 * class knows or cares which branch was taken.
 */
public class RobotContainer {
    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;
    private final Indexer indexer;
    private final Leds leds;
    private final AutoSelector autoSelector;
    private final UsbCamera camera;

    private static final CommandXboxController driver =
            new CommandXboxController(OIConstants.kDriverControllerPort);
    private static final CommandXboxController operator =
            new CommandXboxController(OIConstants.kOperatorControllerPort);

    private final GenericEntry demoToggle;

    public RobotContainer() {
        Flywheel flywheel;
        Hood hood;
        Kicker kicker;
        Pivot pivot;
        Funnel funnel;

        switch (Constants.currentMode) {
            case REAL -> {
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOSpark(0),
                                new ModuleIOSpark(1),
                                new ModuleIOSpark(2),
                                new ModuleIOSpark(3));
                vision = new Vision(drive, new VisionIOLimelight(VisionConstants.kCameraName));
                flywheel = new Flywheel(new FlywheelIOSpark());
                hood = new Hood(new HoodIOSpark());
                kicker = new Kicker(new KickerIOSpark());
                pivot = new Pivot(new PivotIOSpark());
                funnel = new Funnel(new FunnelIOTalonFX());
                indexer = new Indexer(new IndexerIOSpark());
            }
            case SIM -> {
                // The gyro reads its rotation rate from the drivetrain, which does not exist yet.
                // A holder breaks the cycle without giving the gyro a reference to the whole
                // subsystem.
                Drive[] self = new Drive[1];
                drive =
                        new Drive(
                                new GyroIOSim(
                                        () ->
                                                self[0] == null
                                                        ? 0.0
                                                        : self[0].getChassisSpeeds()
                                                                .omegaRadiansPerSecond),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim());
                self[0] = drive;
                vision = new Vision(drive, new VisionIOSim());
                flywheel = new Flywheel(new FlywheelIOSim());
                hood = new Hood(new HoodIOSim());
                kicker = new Kicker(new KickerIOSim());
                pivot = new Pivot(new PivotIOSim());
                funnel = new Funnel(new FunnelIOSim());
                indexer = new Indexer(new IndexerIOSim());
            }
            default -> {
                // REPLAY: every IO is a do-nothing stub. AdvantageKit overwrites the inputs with
                // the values recorded in the log, so the subsystems see exactly what they saw on
                // the field while the control code above them is whatever is checked out now.
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {});
                vision = new Vision(drive, new VisionIO() {});
                flywheel = new Flywheel(new FlywheelIO() {});
                hood = new Hood(new HoodIO() {});
                kicker = new Kicker(new KickerIO() {});
                pivot = new Pivot(new PivotIO() {});
                funnel = new Funnel(new FunnelIO() {});
                indexer = new Indexer(new IndexerIO() {});
            }
        }

        shooter = new Shooter(flywheel, hood, kicker);
        intake = new Intake(pivot, funnel);
        leds = new Leds();

        // The driver camera only exists on the real robot; in simulation CameraServer would open
        // the laptop webcam.
        if (RobotBase.isReal()) {
            camera = CameraServer.startAutomaticCapture(0);
            camera.setFPS(60);
            camera.setResolution(320, 240);
            Shuffleboard.getTab("Main")
                    .add("Camera", camera)
                    .withWidget(BuiltInWidgets.kCameraStream)
                    .withSize(4, 4)
                    .withPosition(2, 0);
        } else {
            camera = null;
        }

        demoToggle =
                Shuffleboard.getTab("Main")
                        .add("Demo Mode", false)
                        .withWidget(BuiltInWidgets.kToggleSwitch)
                        .withSize(1, 1)
                        .withPosition(9, 0)
                        .getEntry();

        registerNamedCommands();

        autoSelector = new AutoSelector();
        autoSelector.addOption(
                "Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

        configureBindings();
    }

    /** Commands PathPlanner autos can call by name. */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("Intake", intake.getFunnel().run(false));
        NamedCommands.registerCommand("Index", indexer.run(false));
        NamedCommands.registerCommand(
                "PivotUp", intake.getPivot().goToAndWait(IntakeConstants.kPivotStowedRad));
        NamedCommands.registerCommand(
                "PivotMiddle", intake.getPivot().goToAndWait(IntakeConstants.kPivotMiddleRad));
        NamedCommands.registerCommand(
                "PivotDown", intake.getPivot().goToAndWait(IntakeConstants.kPivotDeployedRad));
        NamedCommands.registerCommand("Feed", intake.getPivot().shake());
        NamedCommands.registerCommand(
                "ShootMiddle",
                Commands.parallel(shooter.getFlywheel().runAtRpm(3000), indexer.run(false)));
        NamedCommands.registerCommand(
                "CalculatedShoot",
                Commands.parallel(shooter.rangedShot(drive::getDistanceToHub), indexer.run(false)));
        NamedCommands.registerCommand(
                "Shoot", Commands.parallel(shooter.shuffleShot(), indexer.run(false)));
        NamedCommands.registerCommand(
                "AutoAlign",
                DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> 0.0,
                        () -> 0.0,
                        drive::getAngleToHub,
                        () -> false,
                        () -> false));
    }

    private void configureBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        driver.getHID()::getRightBumperButton,
                        driver.getHID()::getXButton));

        // The kicker's default command is the flywheel interlock: it feeds when, and only when,
        // both shooter wheels are at speed. Every shooting command therefore gets the interlock
        // for free rather than having to remember it.
        shooter.getKicker()
                .setDefaultCommand(shooter.getKicker().feedWhen(shooter.getFlywheel()::atSpeed));

        driver.back().onTrue(Commands.runOnce(drive::resetHeading).ignoringDisable(true));
        driver.rightTrigger()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -driver.getLeftY(),
                                () -> -driver.getLeftX(),
                                drive::getAngleToHub,
                                driver.getHID()::getRightBumperButton,
                                driver.getHID()::getXButton));

        // Slow mode buzzes the driver's controller so they can feel that it is engaged. This used
        // to be done by poking setRumble() from inside the drive command's execute(); as a command
        // with no requirements it composes with everything else instead of being duplicated in
        // each drive mode.
        driver.rightBumper()
                .whileTrue(DriveCommands.rumble(driver.getHID()).withName("Slow mode rumble"));

        operator.leftBumper().whileTrue(intake.getFunnel().run(false));
        operator.b().whileTrue(intake.getFunnel().run(true));
        operator.leftTrigger().whileTrue(indexer.run(false));

        operator.povUp().whileTrue(shooter.getHood().trim(false));
        operator.povDown().whileTrue(shooter.getHood().trim(true));

        operator.rightTrigger()
                .whileTrue(
                        Commands.parallel(
                                shooter.rangedShot(drive::getDistanceToHub), indexer.run(false)));
        operator.rightBumper()
                .whileTrue(Commands.parallel(shooter.shuffleShot(), indexer.run(false)));

        operator.y().whileTrue(intake.getPivot().goTo(IntakeConstants.kPivotStowedRad));
        operator.x().whileTrue(intake.getPivot().goTo(IntakeConstants.kPivotMiddleRad));
        operator.a().whileTrue(intake.getPivot().goTo(IntakeConstants.kPivotDeployedRad));
        operator.povLeft().whileTrue(intake.getPivot().shake());
    }

    /**
     * Called once per loop from {@link Robot}: read the dashboard toggles into {@link RobotState}.
     */
    public void updateDashboardInputs() {
        RobotState.getInstance().setDemoMode(demoToggle.getBoolean(false));
    }

    public Command getAutonomousCommand() {
        return autoSelector.get();
    }

    public void setDisabledMode(boolean disabled) {
        vision.setDisabledMode(disabled);
    }

    public static XboxController getDriverJoystick() {
        return driver.getHID();
    }

    // Accessors used by the simulation smoke test.
    public Drive getDrive() {
        return drive;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Intake getIntake() {
        return intake;
    }

    public Indexer getIndexer() {
        return indexer;
    }
}
