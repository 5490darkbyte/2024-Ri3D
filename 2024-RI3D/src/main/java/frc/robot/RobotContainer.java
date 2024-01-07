package frc.robot;

import frc.robot.subsystems.climber.Climber;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.utility.RobotIdentity;
import frc.robot.utility.SubsystemFactory;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.*;

public class RobotContainer {

  // Creating Controlers
  @SuppressWarnings({ "unused" })
  private final CommandXboxController driveController = new CommandXboxController(DRIVE_CONTROLLER_PORT);
  @SuppressWarnings({ "unused" })
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  @SuppressWarnings({ "unused" })
  private final CommandXboxController testController = new CommandXboxController(TEST_CONTROLLER_PORT);

  private DriveTrain driveTrainSubsystem;

  private Climber ClimberSubsystem;

  private DriveCommand driveCommand;

  private ClimberCommand climberCommand;

  private RobotIdentity identity;

  public RobotContainer() {
    identity = RobotIdentity.getIdentity();

    createSubsystems(); // Create our subsystems.
    createCommands(); // Create our commands
    configureButtonBindings(); // Configure the button bindings
  }

  private void createSubsystems() {

    driveTrainSubsystem = SubsystemFactory.createDriveTrain(identity);
    ClimberSubsystem = SubsystemFactory.createClimber(identity);
  }

  private void createCommands() {

    driveCommand = new DriveCommand(driveTrainSubsystem,
        () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis(),
        () -> -driveController.getLeftX());
    driveTrainSubsystem.setDefaultCommand(driveCommand);
    
  }

  private void configureButtonBindings() {

    // Toggle Brake Mode with A
    driveController.a().onTrue(new InstantCommand(() -> driveTrainSubsystem.toggleMode(), driveTrainSubsystem));
    driveController.a().onTrue(new ClimberCommand(ClimberSubsystem));
  }


  public Command getAutonomousCommand() {
    return null;
  }
}