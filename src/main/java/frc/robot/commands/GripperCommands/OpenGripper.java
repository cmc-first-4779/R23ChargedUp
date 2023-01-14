// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GripperCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class OpenGripper extends CommandBase {
  /** Creates a new OpenGripper. */

  Gripper m_gripper;

 

  public OpenGripper() {
    m_gripper = new Gripper();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.OpenGripper();
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
