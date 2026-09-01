// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.intakeConstants;
import frc.robot.Constants.shooterConstants;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.shooterConstants.*;

public class Intake extends SubsystemBase {
  private SparkMax shooter;
  private SparkMaxConfig shooterConfig;
  private SparkMax feeder;
  private SparkMaxConfig feederConfig;
  private SparkFlex intake;
  private SparkClosedLoopController sparkControl;
  private RelativeEncoder encoder;
  private RelativeEncoder shooterEncoder;
  private SysIdRoutine shooterSysID;
  private SparkFlexConfig intakeConfig;

  /** Creates a new Intake. */
  public Intake() { 
    shooter = new SparkMax(shooterConstants.SHOOTER, MotorType.kBrushless);
    feeder = new SparkMax(intakeConstants.FEEDER_ID, MotorType.kBrushless);
    intake = new SparkFlex(9,MotorType.kBrushless );

    shooterConfig = new SparkMaxConfig();
    feederConfig = new SparkMaxConfig();
    intakeConfig = new SparkFlexConfig();
    
    sparkControl = intake.getClosedLoopController();
    encoder = intake.getEncoder();
    shooterEncoder = shooter.getEncoder();
    
    feederConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60);
    
    intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
//PID woot! woot!
    shooterConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60)
      .closedLoop.positionWrappingEnabled(true);
      // .pid(0, 0, 0);

    shooterConfig.closedLoop.feedForward
      .sva(0.16714, 0.00034167, 0);

    shooterConfig.encoder.velocityConversionFactor(VELOCITY_CONVERT);
    
//config
    shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterSysID = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(2).per(Second),
        Volts.of(8),
        Seconds.of(30)
      ), 
      new SysIdRoutine.Mechanism(
        (volts) -> shooter.setVoltage(volts.in(Volts)), 
        null,
        this));
  }

  /** spins the intake */
  public void spinIntake(double spinSpeed, double feederSpeed){
    shooter.set(spinSpeed);
    intake.set(spinSpeed);
    feeder.set(feederSpeed);
  }

  /** spins the intake with joystick */
  // public Command intakeWithJoystick(double speed){
  //   return Commands.run(() ->  {shooter.set(speed);}, this);
  // }

  /**
 * creates the stop command to prevent continuous running. This will becalled upon during autos to end shoot commandss to prevent constantoutput
 */
  public void stop(){
    intake.stopMotor();
    shooter.stopMotor();
    feeder.stopMotor();
  }

  /**This command is largely empty
   * placed to prevent an ERROR (/°W^)
   */
  // public void spinShoot(){
  //   shooter.set(.4);
  // }

  /**will be pulled from when firing
   * 
   */
  public void PIDShoot(double fireSpeed){
    feeder.set(-.6);
    shooter.set(0.8);
    ///sparkControl.setSetpoint(fireSpeed, ControlType.kVelocity);
  }

  // public void intakeMotorShooter(){
  //   feeder.set(.4);
  // }

  public void spinShooter() {//both spinshooter and spinfeeder are relavent to shoot withdelay as they isolate the uses allowing for the shooter to be spun without allowing for the feeder to cause clogging.
    shooter.set(0.8);
  }
  public void spinFeeder() {
    feeder.set(-0.6);
  }

  public Command runSysID(){
    return Commands.sequence(
      shooterSysID.quasistatic(Direction.kForward).withTimeout(5),
      new WaitCommand(2),
      shooterSysID.quasistatic(Direction.kReverse).withTimeout(5),
      new WaitCommand(2),
      shooterSysID.dynamic(Direction.kForward).withTimeout(5),
      new WaitCommand(2),
      shooterSysID.dynamic(Direction.kReverse).withTimeout(5));
  }
/**return shooter velocity*/
  public double getVelocity(){
    return encoder.getVelocity();
  }
/**gets the voltage applied to the motor*/
  public double getVoltage(){
    return shooter.getAppliedOutput()*shooter.getBusVoltage();
  }
/**stops the shoot motor*/
  public void intakeMotorShooterStop(){
    feeder.stopMotor();
  }
/**applies the velocity constant to the spped value*/
  public void intakeWithPID(double speed){
    sparkControl.setSetpoint(speed, ControlType.kVelocity);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
/**elastic stuffs*/
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("shooter velocity", () -> shooterEncoder.getVelocity(), null);
    builder.addDoubleProperty("Shooter Voltage", () -> getVoltage(), null);
    builder.addDoubleProperty("Shooter Encoder Position", () -> shooterEncoder.getPosition(), null);

  }
}
