// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BlingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlingSubsystem;

public class BlingSetPatternCommand extends CommandBase {
  /** Creates a new BlingSetPatternCommand. */

  public final BlingSubsystem m_blingSubsystem;
  public final double m_pattern;

  public BlingSetPatternCommand(BlingSubsystem blingSubsystem, double pattern) {
    m_blingSubsystem = blingSubsystem; // Bring the Subsystem from the constructor to this class.
    m_pattern = pattern; // Bring the pattern from the constructor to this class.
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_blingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the bling pattern by calling the setBlingPattern
    m_blingSubsystem.setBlingPattern(m_pattern);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
