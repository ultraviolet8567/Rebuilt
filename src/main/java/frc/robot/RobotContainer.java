// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.AutoChooser;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Storage.Storage;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Swerve swerve;
    private final Odometry odometry;
    private final Shooter shooter;
    private final AutoChooser autoChooser;
    private final Intake intake;
    private final Storage storage;
    private final UsbCamera camera;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private static final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private static final CommandXboxController operatorController =
            new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings

        swerve = new Swerve(ModuleConstants.kDriveMotorGearing);
        odometry = new Odometry(swerve);
        shooter = new Shooter();
        intake = new Intake();
        storage = new Storage();
        camera = CameraServer.startAutomaticCapture(0);

        if (RobotBase.isReal()) {
            camera.setFPS(60);
            camera.setResolution(320, 240);
        }

        // Configure the PathPlanner auto-builder
        AutoBuilder.configure(
                odometry::getPose,
                odometry::resetPose,
                swerve::getRobotRelativeSpeeds,
                swerve::setModuleStates,
                AutoConstants.kHolonomicController, // rotational PID
                DriveConstants.kRobotConfig,
                () -> {
                    if (DriverStation.getAlliance().isPresent()) {
                        return DriverStation.getAlliance().get() == Alliance.Red;
                    }
                    return false;
                },
                swerve);

        NamedCommands.registerCommand("Intake", new SpinIntake(intake.getFunnel(), false));
        NamedCommands.registerCommand("Index", new SpinIndexer(storage.getIndexer(), false));
        NamedCommands.registerCommand("ShootMiddle", new Shoot(shooter.getFlywheel(), 3000));
        NamedCommands.registerCommand(
                "PivotUp", new SetPivot(intake.getPivot(), IntakeConstants.kPivotLower));
        NamedCommands.registerCommand(
                "PivotMiddle", new SetPivot(intake.getPivot(), IntakeConstants.kPivotMiddle));
        NamedCommands.registerCommand(
                "PivotDown", new SetPivot(intake.getPivot(), IntakeConstants.kPivotUpper));
        NamedCommands.registerCommand(
                "CalculatedShoot", new CalculatedShoot(shooter.getFlywheel(), odometry));
        NamedCommands.registerCommand(
                "Shoot", new Shuffle(shooter.getFlywheel(), shooter.getHood()));
        NamedCommands.registerCommand(
                "AutoAlign",
                new DriftTeleOp(
                        swerve,
                        odometry,
                        () -> 0.0,
                        () -> 0.0,
                        () -> -odometry.angleToHub().getRadians(),
                        () -> false,
                        () -> false));
        // NamedCommands.registerCommand(
        //        "Kicker", new RunKicker(shooter.getFlywheel(), shooter.getKicker()));

        // Shuffleboard.getTab("Main")
        //        .add(
        //                "Reset Pivot",
        //                new InstantCommand(() -> intake.getPivot().resetRelativeEncoder()))
        //        .withWidget(BuiltInWidgets.kCommand);

        autoChooser = new AutoChooser();

        Shuffleboard.getTab("Main")
                .add("Camera", camera)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withSize(4, 4)
                .withPosition(2, 0);

        swerve.setDefaultCommand(
                new ManualTeleOp(
                        swerve,
                        odometry,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> driverController.getHID().getRightBumperButton(),
                        () -> driverController.getHID().getXButton()));

        shooter.getKicker()
                .setDefaultCommand(new RunKicker(shooter.getFlywheel(), shooter.getKicker()));

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        driverController.back().onTrue(new InstantCommand(() -> odometry.resetHeading()));
        driverController
                .rightTrigger()
                .whileTrue(
                        new DriftTeleOp(
                                swerve,
                                odometry,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                () -> -odometry.angleToHub().getRadians(),
                                () -> driverController.getHID().getRightBumperButton(),
                                () -> driverController.getHID().getXButton()));

        // driverController.x().onTrue(new LockWheels(swerve));
        operatorController.leftBumper().whileTrue(new SpinIntake(intake.getFunnel(), false));
        operatorController.leftTrigger().whileTrue(new SpinIndexer(storage.getIndexer(), false));
        // operatorController.rightBumper().whileTrue(new ManualKicker(shooter.getKicker()));
        operatorController.povUp().whileTrue(new SetHood(shooter.getHood(), false));
        operatorController.povDown().whileTrue(new SetHood(shooter.getHood(), true));
        // operatorController.povRight().whileTrue(new ManualKicker(shooter.getKicker()));
        operatorController
                .rightTrigger()
                .whileTrue(new CalculatedShoot(shooter.getFlywheel(), odometry));
        operatorController
                .rightBumper()
                .whileTrue(new Shuffle(shooter.getFlywheel(), shooter.getHood()));

        operatorController
                .y()
                .whileTrue(new SetPivot(intake.getPivot(), IntakeConstants.kPivotLower));
        operatorController
                .x()
                .whileTrue(new SetPivot(intake.getPivot(), IntakeConstants.kPivotMiddle));
        operatorController
                .a()
                .whileTrue(new SetPivot(intake.getPivot(), IntakeConstants.kPivotUpper));
        operatorController.b().whileTrue(new SpinIntake(intake.getFunnel(), true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelectedAuto();
    }

    public static XboxController getDriverJoystick() {
        return driverController.getHID();
    }
}
