// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TimedDrive extends Command {
  private TankDrive drive;
  private double seconds;
  private Timer timer;

  /** Creates a new TimeDrive. */
  public TimedDrive(TankDrive drive, double seconds) {
    this.drive = drive;
    this.seconds = seconds;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Resets the timer to 0 when called
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Drives for a certain amount of time
    drive.drive(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops driving when command ends
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // When the amount of time set has passed, it will return true which ends the command
    return timer.get() > seconds;
  }
}
