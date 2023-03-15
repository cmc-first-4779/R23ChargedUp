// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightSetLEDMode extends CommandBase {
  /** Creates a new LimelightSetLEDModeCommand. */
  LimelightSubsystem m_limelightSubsystem;
  double m_LEDMode;
  
  public LimelightSetLEDMode(LimelightSubsystem limelightSubsystem, double LEDMode) {
    m_limelightSubsystem = limelightSubsystem;
    m_LEDMode = LEDMode;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelightSubsystem.setLEDMode(m_LEDMode);
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