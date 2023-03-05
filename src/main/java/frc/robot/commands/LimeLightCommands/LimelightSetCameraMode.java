// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightSetCameraMode extends CommandBase {
  /** Creates a new LimelightSetCameraModeCommand. */
  LimelightSubsystem m_limelightSubsystem;
  double m_camMode;

  public LimelightSetCameraMode(LimelightSubsystem limelightSubsystem, double camMode) {
    m_limelightSubsystem = limelightSubsystem;
    m_camMode = camMode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelightSubsystem.setCameraMode(m_camMode);
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