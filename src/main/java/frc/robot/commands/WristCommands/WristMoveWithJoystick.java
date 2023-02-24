// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.WristSubsystem;

public class WristMoveWithJoystick extends CommandBase {
  // Declare our subsystem
  WristSubsystem wristSubsystem;
  CommandPS4Controller joystick;

  /** Creates a new WristGroundCommand. */
  public WristMoveWithJoystick(WristSubsystem wristSubsystem, CommandPS4Controller joystick) {
    this.wristSubsystem = wristSubsystem;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pass the left joystick's Y value to the wrist. 
    wristSubsystem.moveWrist(joystick.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stopMotor();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
