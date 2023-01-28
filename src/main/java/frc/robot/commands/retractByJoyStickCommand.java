// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ExtenderSubsystem;

public class retractByJoyStickCommand extends CommandBase {
  ExtenderSubsystem m_extenderSubsystem;

  /** Creates a new lowExtenderCommand. */
  public retractByJoyStickCommand(ExtenderSubsystem extenderSubsystem) {
    m_extenderSubsystem = extenderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_extenderSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_extenderSubsystem.retractArm();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extenderSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
