// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.shooterConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.studica.frc.AHRS;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj.MotorSafety;

public class TankDrive extends SubsystemBase {
  // Creates new objects
    private SparkMax frontLeft;
    private SparkMax frontRight;
    private SparkMax backLeft;
    private SparkMax backRight;
    private DifferentialDrive tankDrive;
    private SparkMaxConfig frontLeftConfig;
    private SparkMaxConfig frontRightConfig;
    private SparkMaxConfig backLeftConfig;
    private SparkMaxConfig backRightConfig;
    private SysIdRoutine driveSysID;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder frontRightEncoder;
    private AHRS NavX;
    private SparkClosedLoopController frontLeftPID;
    private SparkClosedLoopController frontRightPID;
    private DifferentialDriveKinematics kinematics; 
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private ChassisSpeeds chassisSpeeds;

  /** Creates a new TankDrive. */
  public TankDrive() {
    // This is creating and configuring the motors
    frontLeft = new SparkMax(FRONT_LEFT_MOTOR, MotorType.kBrushless); 
    frontRight = new SparkMax(FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backLeft = new SparkMax(BACK_LEFT_MOTOR, MotorType.kBrushless);
    backRight = new SparkMax(BACK_RIGHT_MOTOR, MotorType.kBrushless);
    frontLeftPID = frontLeft.getClosedLoopController();
    frontRightPID = frontRight.getClosedLoopController();

    frontLeftConfig = new SparkMaxConfig();
    frontRightConfig = new SparkMaxConfig();
    backLeftConfig = new SparkMaxConfig();
    backRightConfig = new SparkMaxConfig();
    NavX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    tankDrive = new DifferentialDrive(frontLeft, frontRight);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21));
    wheelSpeeds = new DifferentialDriveWheelSpeeds(LEFT_WHEEL_CONVERT, RIGHT_WHEEL_CONVERT); // Left and right meters per second
    chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    driveEncoder = frontLeft.getEncoder();
    frontRightEncoder = frontRight.getEncoder();

    // Tells the motors how to stop and also inverts one wheel so they both spin the same direction
    frontLeftConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60)
      .inverted(true);

    frontRightConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60);

    //frontLeftConfig.closedLoopRampRate(1);
    //frontRightConfig.closedLoopRampRate(1);

    // frontLeftConfig.closedLoop
    //   .pid(0, 0, 0);

    // frontRightConfig.closedLoop
    //   .pid(0, 0, 0);

    frontLeftConfig.closedLoop.feedForward
      .sva(0.21427, 0.0018127, 0);

    frontRightConfig.closedLoop.feedForward
      .sva(0.1894, 0.0019739, 0);

    // Makes the back left and right wheels follow the front left and right
    backLeftConfig.apply(frontLeftConfig).follow(FRONT_LEFT_MOTOR);
    backRightConfig.apply(frontRightConfig).follow(FRONT_RIGHT_MOTOR);

    frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeft.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRight.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveSysID = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(0.5).per(Second),
        Volts.of(2),
        Seconds.of(30)
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> setDriveVoltage(volts.in(Volts)),
        null,
        this));

    NavX.setAngleAdjustment(10);
  }

  public void setDriveVoltage(double voltage){
    frontLeft.setVoltage(voltage);
    frontRight.setVoltage(voltage);
  }

  public Command runSysID(){
    return Commands.sequence(
      driveSysID.quasistatic(Direction.kReverse).withTimeout(3),
      driveSysID.quasistatic(Direction.kForward).withTimeout(3),
      driveSysID.dynamic(Direction.kReverse).withTimeout(3),
      driveSysID.dynamic(Direction.kForward).withTimeout(3));
  }

  public double getGyroReading(){
    return NavX.getAngle();
    // return NavX.getYaw();
  }

  public void driveWithRotateLock(double speed, double angle){
    PIDController rotateController = new PIDController(0.0075, 0, 0.002);
    double rotateSpeed = rotateController.calculate(getGyroReading(), angle);
    rotateSpeed = MathUtil.clamp(rotateSpeed, -1, 1);
    tankDrive.arcadeDrive(speed, rotateSpeed);
  }

  // public static double cubicDrive(double value){
  //   return Math.pow(value, 3);
  //   //value*value*value;
  // }
  


  /** Makes the robot drive with joystick */
  // public void joystickDrive(XboxController driver){
  //   // tankDrive.tankDrive(MathUtil.applyDeadband(-driver.getLeftY(), 0.05), MathUtil.applyDeadband(-driver.getRightY(), 0.05));
  //   // tankDrive.tankDrive(cubicDrive(-driver.getLeftY()), cubicDrive(-driver.getRightY()));
  //   // tankDrive.arcadeDrive(MathUtil.applyDeadband(driver.getLeftY(), 0.05)/1.4, (MathUtil.applyDeadband(driver.getRightX(), 0.05))/1.35);
  //   double leftSideSetpoint = (MathUtil.applyDeadband(driver.getLeftY(), 0.05)/1.4) + (MathUtil.applyDeadband(driver.getRightX(), 0.05)/1.35);  // * 3000 if doesnt work
  //   double rightSideSetpoint = (MathUtil.applyDeadband(driver.getLeftY(), 0.05)/1.4) - (MathUtil.applyDeadband(driver.getRightX(), 0.05)/1.35);

  //   frontLeftPID.setSetpoint(leftSideSetpoint, ControlType.kVelocity);
  //   frontRightPID.setSetpoint(rightSideSetpoint, ControlType.kVelocity);
  //   //tankDrive.arcadeDrive(cubicDrive(-driver.getLeftY()), cubicDrive(-driver.getRightX()));
  // }

  public void squareJoystickDrive(XboxController driver){
    double driveSpeed = MathUtil.applyDeadband(driver.getLeftY()/1.2 * driver.getLeftY()/1.2 * Math.signum(driver.getLeftY()), 0.05);
    double rotateSpeed = MathUtil.applyDeadband(driver.getRightX()/1.5, 0.05);
    tankDrive.arcadeDrive(driveSpeed, rotateSpeed);
  }

  /** Drive and rotate at the given speeds */
  public void drive(double speed){
    // tankDrive.arcadeDrive(speed * 1.5 , rotationSpeed * .5);\
    setSetpoint(speed);
  }

  public void setSetpoint(double setpoint){
    frontLeftPID.setSetpoint(setpoint, ControlType.kVelocity);
    frontRightPID.setSetpoint(setpoint, ControlType.kVelocity);
  }

  public Command timedDrive(double speed, double time) {
    //return Commands.run(() -> tankDrive.arcadeDrive(speed, 0), this).withTimeout(time);
    return Commands.run(() -> move(speed), this).withTimeout(time);
  }

  public void move(double speed) {
    frontLeft.set(speed);
    frontRight.set(speed);
  }

  /** Calls to stop the motors */
  public void stop(){
    frontLeft.stopMotor();
    frontRight.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("TankDrive");

    builder.addDoubleProperty("FL Voltage", () -> frontLeft.getAppliedOutput() * RobotController.getBatteryVoltage(), null);
    builder.addDoubleProperty("FR Voltage", () -> frontRight.getAppliedOutput() * RobotController.getBatteryVoltage(), null);
    builder.addDoubleProperty("BL Voltage", () -> backLeft.getAppliedOutput() * RobotController.getBatteryVoltage(), null);
    builder.addDoubleProperty("BR Voltage", () -> backRight.getAppliedOutput() * RobotController.getBatteryVoltage(), null);

    builder.addDoubleProperty("FR Current", () -> frontRight.getOutputCurrent(), null);
    builder.addDoubleProperty("FL Current", () -> frontLeft.getOutputCurrent(), null);
    builder.addDoubleProperty("BR Current", () -> backRight.getOutputCurrent(), null);
    builder.addDoubleProperty("BL Current", () -> backLeft.getOutputCurrent(), null);

    builder.addDoubleProperty("Drive velocity", () -> driveEncoder.getVelocity(), null);
    builder.addDoubleProperty("Drive position", () -> driveEncoder.getPosition(), null);
    builder.addDoubleProperty("FR Drive Velocity", () -> frontRightEncoder.getVelocity(), null);
    builder.addDoubleProperty("FR Drive Position", () -> frontRightEncoder.getPosition(), null);

    builder.addDoubleProperty("Gyro reading", () -> getGyroReading(), null);
  }
}
