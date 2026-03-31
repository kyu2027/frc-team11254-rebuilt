  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// imports (figure it out)
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //define stuff
  private TankDrive drive;
  private Intake intakeSubsystem;
  
  private XboxController driver;
  private XboxController operator;
  private climber climbSubsystem;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    // Setting the Xbox Controller to the driver port
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private JoystickButton intakeButton;
  private JoystickButton outakeButton;
  private JoystickButton shootButton;
  private JoystickButton unstuckButton;
  private JoystickButton climbupButton;
  private JoystickButton climbdownButton;

  private Command driveWithJoystick;
  private Command spinIntake;
  private Command outake;
  private Command shootCommand;
  private Command unstuckinator;
  private Command shooterIntake;
  private Command climbUp;
  private Command Climbdown;
  private UsbCamera camera;

  private SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * here is where speeds are set as well as the controll devices assigned
   * RIGHT bumper to shoot LEFT bumper to move motor in reverse
  */
  public RobotContainer() {
    intakeSubsystem = new Intake();
    drive = new TankDrive();
    climbSubsystem = new climber();
    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 420);
    camera.setFPS(30);
    
    spinIntake = Commands.runEnd(() -> {intakeSubsystem.spinIntake(0.5, 0.8);}, ()-> {intakeSubsystem.spinIntake(0,0);}, intakeSubsystem);
    outake = Commands.runEnd(() -> {intakeSubsystem.spinIntake(-0.6, -0.8);}, () -> {intakeSubsystem.spinIntake(0,0);}, intakeSubsystem);
    shootCommand = Commands.runEnd(() -> intakeSubsystem.PIDShoot(3000), () -> intakeSubsystem.stop(),  intakeSubsystem);
    unstuckinator = Commands.runEnd(() -> intakeSubsystem.PIDShoot(500), () -> intakeSubsystem.stop(), intakeSubsystem);
    driveWithJoystick = Commands.run(() -> drive.joystickDrive(driver), drive);
    climbUp = Commands.run(() -> climbSubsystem.climbup(), climbSubsystem);
    Climbdown = Commands.run(() -> climbSubsystem.climbdown(), climbSubsystem);

    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Intake SysID", Autos.runIntakeSysID(intakeSubsystem));
    autoChooser.addOption("Center Auto", Autos.centerAuto(drive, intakeSubsystem));
    autoChooser.addOption("Side Auto", Autos.sideAuto(drive,intakeSubsystem));
    autoChooser.addOption("Climb Auto", Autos.climbAuto(drive, climbSubsystem, climbSubsystem));
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(climbSubsystem);
    SmartDashboard.putData(intakeSubsystem);

    driver = new XboxController(0);
    operator = new XboxController(1);
    
    intakeButton = new JoystickButton(operator, XboxController.Button.kA.value);
    outakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
    unstuckButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value); //left bumper
    shootButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    climbupButton = new JoystickButton(operator, XboxController.Button.kX.value);
    climbdownButton = new JoystickButton(operator, XboxController.Button.kY.value);

    SmartDashboard.putData(drive);
    
    // Configure the trigger bindings
    drive.setDefaultCommand(driveWithJoystick);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    intakeButton.whileTrue(spinIntake); // a button
    outakeButton.whileTrue(outake); // b button
    shootButton.whileTrue(shootCommand); //right bumper
    climbupButton.onTrue(climbUp);
    climbdownButton.onTrue(Climbdown);

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
