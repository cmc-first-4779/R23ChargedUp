// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtenderCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtendByJoystick extends CommandBase {
  /** Creates a new ExtendByJoystick. */
  ExtenderSubsystem extenderSubsystem;
  CommandPS4Controller joystick;

  public ExtendByJoystick(ExtenderSubsystem extenderSubsystem, CommandPS4Controller joystick) {
    this.extenderSubsystem = extenderSubsystem;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Put the encoder value of the Master Motor to the Dashboard
    // SmartDashboard.putNumber("ExtendEffort", joystick.getLeftY());
    extenderSubsystem.moveExtender(-joystick.getLeftY());
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
