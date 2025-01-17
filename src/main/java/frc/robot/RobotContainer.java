package frc.robot;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.UnloadCommand;
import frc.robot.commands.WaitCommandWithExit;
import frc.robot.lib.config.CTREConfigs;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.AutoCommandChooser;
import frc.robot.utility.RobotIdentity;
import frc.robot.utility.SubsystemFactory;
import frc.robot.Camera;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static frc.robot.Constants.*;
import java.security.PublicKey;

@SuppressWarnings("unused")
public class RobotContainer {

  // Creating Xbox Controllers
  @SuppressWarnings({ "unused" })
  private final CommandXboxController driveController = new CommandXboxController(DRIVE_CONTROLLER_PORT);
  @SuppressWarnings({ "unused" })
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  @SuppressWarnings({ "unused" })
  private final CommandXboxController testController = new CommandXboxController(TEST_CONTROLLER_PORT);

  //Declare Subsystem variables
  private Swerve swerveSubsystem;
  private Intake intakeSubsystem;
  private Shooter shooterSubsystem;
  private Arm armSubsystem;
  private Climber climberSubsystemL;
  private Climber climberSubsystemR;
  //Declare autochooser for selecting autonomous code
  private AutoCommandChooser autoChooser;

  //Declare camera variable
  @SuppressWarnings("unused")
  private Camera camera;

  //Declare sequential command groups
  //public DriveCommand defaultDriveCommand;
  //public TurboDriveCommand turboDriveCommand;
  public TeleopSwerve defaultDriveCommand;
  public TeleopSwerve turboDriveCommand;
  private SequentialCommandGroup shootRing;
  private SequentialCommandGroup intakeRing;
  private SequentialCommandGroup hiIntake;
  private SequentialCommandGroup climber;
  private SequentialCommandGroup feederPlace;
  private SequentialCommandGroup shootRingAuto1;
  private SequentialCommandGroup shootRingAuto2;
  private SequentialCommandGroup shootRingAuto3;
  private SequentialCommandGroup autoShootDrive;
  private SequentialCommandGroup autoAmpBlue;
  private SequentialCommandGroup autoAmpRed;
  private SequentialCommandGroup feedBack;
  private SequentialCommandGroup feedForward;
     
  //Declare robot identity
  private RobotIdentity identity;

  public RobotContainer() {
    //Initialize robot identity (real or simulation)
    identity = RobotIdentity.getIdentity();

    createSubsystems(); // Initialize all the subsystems
    createCommands(); // Initialize Command groups and add commands to them
    configureButtonBindings(); // Configure the button bindings
    createAutoCommand(); //Register autonomous commands and put the chooser in shuffleboard
  }


  //Initialize subsystems
  private void createSubsystems() {
    swerveSubsystem = SubsystemFactory.createSwerve(identity);
    climberSubsystemL = SubsystemFactory.createClimber(identity, Constants.ARM_MOTOR_LEFT);
    climberSubsystemR = SubsystemFactory.createClimber(identity, Constants.ARM_MOTOR_RIGHT);
    intakeSubsystem = SubsystemFactory.createIntake(identity);
    shooterSubsystem = SubsystemFactory.createShooter(identity);
    armSubsystem = SubsystemFactory.createArm(identity);
    camera = SubsystemFactory.createCamera(identity);
  }


  // Initialize Command groups and add commands to them
  private void createCommands() {
    /* 
    defaultDriveCommand = new DriveCommand(driveTrainSubsystem,
        () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis(),
        () -> -driveController.getLeftX());

    turboDriveCommand = new TurboDriveCommand(driveTrainSubsystem,
        () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis(),
        () -> -driveController.getLeftX());

    driveTrainSubsystem.setDefaultCommand(defaultDriveCommand);
    */

    defaultDriveCommand = new TeleopSwerve(swerveSubsystem, 
        () -> driveController.getLeftY(), 
        () -> driveController.getLeftX(),
        () -> driveController.getRightX(),
        () -> driveController.rightBumper().getAsBoolean(),
        Constants.SwerveConstants.defaultMaxSpeed, 1.0, 1.0);

    swerveSubsystem.setDefaultCommand(defaultDriveCommand);

    turboDriveCommand = new TeleopSwerve(swerveSubsystem, 
        () -> driveController.getLeftY(), 
        () -> driveController.getLeftX(),
        () -> driveController.getRightX(),
        () -> driveController.rightBumper().getAsBoolean(),
        Constants.SwerveConstants.turboMaxSpeed, 1.0, 1.0);


    shootRing = new SequentialCommandGroup();  
    //shootRing.addCommands(new InstantCommand(() -> driveTrainSubsystem.setLowCurrentMode()));
    shootRing.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
    shootRing.addCommands(new InstantCommand(() -> shooterSubsystem.setPID(Units.rotationsPerMinuteToRadiansPerSecond(4500.0))));
    shootRing.addCommands(new WaitCommandWithExit(1.5, () -> driveController.b().getAsBoolean()));
    shootRing.addCommands(new UnloadCommand(intakeSubsystem, () -> driveController.b().getAsBoolean()));
    //shootRing.addCommands(new InstantCommand(() -> driveTrainSubsystem.setHighCurrentMode()));
    shootRing.addCommands(new StopShooterCommand(shooterSubsystem));

    intakeRing = new SequentialCommandGroup();
    intakeRing.addCommands(new InstantCommand(() -> armSubsystem.setPosition("INTAKE")));
    intakeRing.addCommands(new IntakeCommand(intakeSubsystem, () -> driveController.b().getAsBoolean(), 0.8));
    intakeRing.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));

    hiIntake = new SequentialCommandGroup();
    hiIntake.addCommands(new InstantCommand(() -> armSubsystem.setPosition("HI_INTAKE")));
    hiIntake.addCommands(new IntakeCommand(intakeSubsystem, () -> driveController.b().getAsBoolean(), 0.4));
    hiIntake.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));

    feederPlace = new SequentialCommandGroup();
    feederPlace.addCommands(new InstantCommand(() -> armSubsystem.setPosition("AMP")));
    feederPlace.addCommands(new WaitCommand(1));
    feederPlace.addCommands(new UnloadCommand(intakeSubsystem, () -> driveController.b().getAsBoolean()));
    feederPlace.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));

    climber = new SequentialCommandGroup();
    climber.addCommands(new InstantCommand(() -> armSubsystem.setPosition("CLIMB")));
    climber.addCommands(new ClimberCommand(climberSubsystemL, () -> driveController.getRightY(), armSubsystem,() -> driveController.b().getAsBoolean()));

    feedBack = new SequentialCommandGroup();
    feedBack.addCommands(new InstantCommand(() -> intakeSubsystem.setPower(-.2)));
    feedBack.addCommands(new WaitCommand(0.025));
    feedBack.addCommands(new InstantCommand(() -> intakeSubsystem.setPower(0.0)));

    feedForward = new SequentialCommandGroup();
    feedForward.addCommands(new InstantCommand(() -> intakeSubsystem.setPower(0.2)));
    feedForward.addCommands(new WaitCommand(0.025));
    feedForward.addCommands(new InstantCommand(() -> intakeSubsystem.setPower(0.0)));

    shootRingAuto1 = new SequentialCommandGroup();
    shootRingAuto1.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
    shootRingAuto1.addCommands(new InstantCommand(() -> shooterSubsystem.setPID(Units.rotationsPerMinuteToRadiansPerSecond(4500.0))));
    shootRingAuto1.addCommands(new WaitCommand(1.5));
    shootRingAuto1.addCommands(new UnloadCommand(intakeSubsystem, () -> false));
    shootRingAuto1.addCommands(new StopShooterCommand(shooterSubsystem));

    shootRingAuto2 = new SequentialCommandGroup();
    shootRingAuto2.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
    shootRingAuto2.addCommands(new InstantCommand(() -> shooterSubsystem.setPID(Units.rotationsPerMinuteToRadiansPerSecond(4500.0))));
    shootRingAuto2.addCommands(new WaitCommand(1.5));
    shootRingAuto2.addCommands(new UnloadCommand(intakeSubsystem, () -> false));
    shootRingAuto2.addCommands(new StopShooterCommand(shooterSubsystem));

    shootRingAuto3 = new SequentialCommandGroup();
    shootRingAuto3.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
    shootRingAuto3.addCommands(new InstantCommand(() -> shooterSubsystem.setPID(Units.rotationsPerMinuteToRadiansPerSecond(4500.0))));
    shootRingAuto3.addCommands(new WaitCommand(1.5));
    shootRingAuto3.addCommands(new UnloadCommand(intakeSubsystem, () -> false));
    shootRingAuto3.addCommands(new StopShooterCommand(shooterSubsystem));
    
    autoShootDrive = new SequentialCommandGroup();
    autoShootDrive.addCommands(shootRingAuto1);
    //autoShootDrive.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.25, 0.25), driveTrainSubsystem));
    autoShootDrive.addCommands(new WaitCommand(2));
    //autoShootDrive.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.0, 0.0), driveTrainSubsystem));

    autoAmpBlue = new SequentialCommandGroup();
    autoAmpBlue.addCommands(shootRingAuto2);
    //autoAmpBlue.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.4, 0.4), driveTrainSubsystem));
    autoAmpBlue.addCommands(new WaitCommand(0.5));
    //autoAmpBlue.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(-0.65, 0.65), driveTrainSubsystem));
    autoAmpBlue.addCommands(new WaitCommand(0.7));
    //autoAmpBlue.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.3, 0.3), driveTrainSubsystem));
    autoAmpBlue.addCommands(new WaitCommand(2.5));
    //autoAmpBlue.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.0, 0.0), driveTrainSubsystem));

    autoAmpRed = new SequentialCommandGroup();
    autoAmpRed.addCommands(shootRingAuto3);
    //autoAmpRed.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.4, 0.4), driveTrainSubsystem));
    autoAmpRed.addCommands(new WaitCommand(0.5));
    //autoAmpRed.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.65, -0.65), driveTrainSubsystem));
    autoAmpRed.addCommands(new WaitCommand(.9));
    //autoAmpRed.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.3, 0.3), driveTrainSubsystem));
    autoAmpRed.addCommands(new WaitCommand(2.5));
    //autoAmpRed.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.0, 0.0), driveTrainSubsystem));

  }


  //We need to add button bindings for the drivetrain
  private void configureButtonBindings(){
    driveController.leftBumper().onTrue(shootRing);
    driveController.a().onTrue(intakeRing);

    operatorController.leftBumper().onTrue(climber);
    driveController.y().onTrue(feederPlace);
    driveController.x().onTrue(hiIntake);

    operatorController.povDown().onTrue(feedBack);
    operatorController.povUp().onTrue(feedForward);

    //driveController.rightBumper().whileTrue(turboDriveCommand);
    driveController.povLeft().onTrue(new InstantCommand(() -> armSubsystem.setPosition("LATCH")));
    driveController.povUp().onTrue(new InstantCommand(() -> armSubsystem.setPosition("LATCHSTANDBY")));
    driveController.povDown().onTrue(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
    driveController.povRight().onTrue(new InstantCommand(() -> armSubsystem.setPosition("LATCHAPROCH")));

    driveController.leftStick().whileTrue(defaultDriveCommand);
  }



  //Register autonomous commands and put the chooser in shuffleboard
  private void createAutoCommand(){
    autoChooser = new AutoCommandChooser();

    // Register all the supported auto commands
    autoChooser.registerDefaultCreator("Do Nothing", null);
    autoChooser.registerCreator("Drive Forward", () -> autoShootDrive);
    autoChooser.registerCreator("Amp Side Left (Blue)", () -> autoAmpBlue);
    autoChooser.registerCreator("Amp Side Right (Red)",() -> autoAmpRed);

    // Setup the chooser in shuffleboard
    autoChooser.setup("Auto", 0, 0, 3, 1);

    }



    public Command getAutonomousCommand() {
      return autoChooser.getAutonomousCommand();
    }



    public void resetSubSystems(){
      //Turn off shooter motors
      shooterSubsystem.setPowers(0);

    }
}