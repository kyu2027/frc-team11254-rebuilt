// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootWithDelay;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.climber;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  public static Command runIntakeSysID(Intake intake){
    return intake.runSysID();
  }

  public static Command centerAuto(TankDrive drive, Intake shoot){
    return Commands.sequence(
      drive.timedDrive(-0.2, 1.35),
      new ShootWithDelay(shoot)
    );
  }

  public static Command climbAuto(TankDrive drive, climber climbUp, climber climbDown){
    return Commands.sequence(
      new ClimbUpForAuto(climbUp).withTimeout(2),
      drive.timedDrive(-.2, 3.2).withTimeout(3.2),
      new ClimbDownForAuto(climbDown).withTimeout(2)
    );
  }

  public static Command sideAuto(TankDrive drive, Intake shoot){
    return Commands.sequence(
      drive.timedDrive(-0.15, 1.5), //WAS 0.4
      new ShootWithDelay(shoot)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
