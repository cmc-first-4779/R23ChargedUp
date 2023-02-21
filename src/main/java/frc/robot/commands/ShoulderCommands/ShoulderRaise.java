// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShoulderCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderRaise extends CommandBase {
  ShoulderSubsystem shoulderSubsystem;
  /** Creates a new ShoulderRaise. */
  public ShoulderRaise(ShoulderSubsystem shoulderSubsystem) {
    this.shoulderSubsystem = shoulderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulderSubsystem.raiseShoulder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
