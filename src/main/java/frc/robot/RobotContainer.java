// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
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
        autoChooser = new AutoChooser();

        NamedCommands.registerCommand("Intake", new SpinIntake(intake.getFunnel()));
        NamedCommands.registerCommand("Index", new SpinIndexer(storage.getIndexer(), false));
        NamedCommands.registerCommand("Shoot", new Shoot(shooter.getFlywheel()));
        NamedCommands.registerCommand(
                "PivotUp", new SetPivot(intake.getPivot(), IntakeConstants.kPivotLower));
        NamedCommands.registerCommand(
                "PivotDown", new SetPivot(intake.getPivot(), IntakeConstants.kPivotUpper));

        /*
        swerve.setDefaultCommand(
                new SwerveTeleOp(
                        swerve,
                        odometry,
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> driverController.getRightX(),
                        () -> driverController.getHID().getRightBumperButton(),
                        () -> driverController.getHID().getXButton()));
         */

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
        driverController.back().onTrue(new InstantCommand(() -> odometry.resetGyrometerHeading()));
        // driverController.x().onTrue(new LockWheels(swerve));
        operatorController.leftBumper().whileTrue(new SpinIntake(intake.getFunnel()));
        operatorController.leftTrigger().whileTrue(new SpinIndexer(storage.getIndexer(), false));
        operatorController.rightBumper().whileTrue(new ManualKicker(shooter.getKicker()));
        operatorController.povUp().whileTrue(new SetHood(shooter.getHood(), false));
        operatorController.povDown().whileTrue(new SetHood(shooter.getHood(), true));
        operatorController.rightTrigger().whileTrue(new DirectShoot(shooter.getFlywheel()));

        operatorController
                .y()
                .whileTrue(new SetPivot(intake.getPivot(), IntakeConstants.kPivotLower));
        operatorController
                .x()
                .whileTrue(new SetPivot(intake.getPivot(), IntakeConstants.kPivotMiddle));
        operatorController
                .a()
                .whileTrue(new SetPivot(intake.getPivot(), IntakeConstants.kPivotUpper));
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
