// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BlingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlingSubsystem;

public class BlingSetPattern extends CommandBase {
  // Declare our local subsystem
  BlingSubsystem bling;
  // Declare a double for our bling pattern
  double pattern;

  /** Creates a new BlingSetPattern. */
  public BlingSetPattern(BlingSubsystem bling, double pattern) {
    //Set our local varibles to our pass through variables
    this.bling = bling;
    this.pattern = pattern;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(bling);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bling.setBlingPattern(pattern);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
