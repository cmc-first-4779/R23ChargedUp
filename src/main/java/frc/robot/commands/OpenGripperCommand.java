// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;

public class OpenGripperCommand extends CommandBase {
  GripperSubsystem m_gripperSubsystem; 
  double currentEncoderPosition;
  /** Creates a new OpenGripperCommand. */
  public OpenGripperCommand (GripperSubsystem gripperSubsystem) {
    m_gripperSubsystem = gripperSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(m_gripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentEncoderPosition = m_gripperSubsystem.getEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentEncoderPosition = m_gripperSubsystem.getEncoder(); 
m_gripperSubsystem.openGripper(2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripperSubsystem.stopGripper(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentEncoderPosition >= Constants.GRIPPER_ENCODER_MAX; 
  }
  }

