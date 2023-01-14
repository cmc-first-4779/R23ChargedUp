// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GripperCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class CloseGripper extends CommandBase {
  Gripper m_gripper;
  /** Creates a new CloseGripper. */
  public CloseGripper() {
    m_gripper = new Gripper(); addRequirements(m_gripper);                      
  }
    // Use addRequirements() here to declare subsystem dependencies.}
     
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     m_gripper.CloseGripper();

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
