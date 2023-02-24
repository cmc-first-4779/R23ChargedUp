// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristSetPosition extends CommandBase {
  // Declare our subsystem
  WristSubsystem wristSubsystem;
  double setPoint;

  /** Creates a new WristGroundCommand. */
  public WristSetPosition(WristSubsystem wristSubsystem, double setPoint) {
    this.wristSubsystem = wristSubsystem;
    this.setPoint = setPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.setWristPosition(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Put the target encoder value of the Motor to the Dashboard
    SmartDashboard.putString("Wrist Target Position", Double.toString(setPoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // wristSubsystem.stopMotor();  // Can't stop motor if you want it to get to position.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
