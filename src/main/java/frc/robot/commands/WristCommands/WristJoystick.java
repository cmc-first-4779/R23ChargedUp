// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristJoystick extends CommandBase {
  // Declare our variables
  WristSubsystem wrist;
  PS4Controller operStick;
  double YValue;

  /** Creates a new WristJoystick. */
  public WristJoystick(WristSubsystem wrist, PS4Controller operStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.operStick = operStick;
    addRequirements(wrist);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the y-axis value of the DriverStick Right Joystick
    YValue = operStick.getRightY();
    // Moving the Wrist
    wrist.moveWristMM(YValue);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the y-axis value of the operStick Left and Right Joystick
    YValue = operStick.getRightY();
    // Moving the Wrist
    wrist.moveWristMM(YValue);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
