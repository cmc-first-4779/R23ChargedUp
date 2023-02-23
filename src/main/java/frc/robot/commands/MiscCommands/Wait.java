// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
  // Declare our timer
  Timer timer = new Timer();
  // Declare local variable for keeping track of the time (seconds)
  private double time;

  /** Creates a new Wait. */
  public Wait(double time) {
    // Set our local time to the time that was passed-thru
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset(); // Reset the timer.
    timer.start(); // Start the timer.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop(); // Stop the timer
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If our timer is less than the time we want it to run, keep running the
    // command.
    if (timer.get() < time) {
      return false;
    }
    // Else once we hit our alloted time, exit out of the command.
    else
      return true;
  }
}
