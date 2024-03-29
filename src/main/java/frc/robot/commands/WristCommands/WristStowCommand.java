// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PositionSetpoints;
import frc.robot.subsystems.WristSubsystem;

public class WristStowCommand extends CommandBase {
  // Declare our subsystem
  WristSubsystem wristSubsystem;

  /** Creates a new WristStopCommand. */
  public WristStowCommand(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.setWristPosition(PositionSetpoints.WRIST_POSITION_STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Put the encoder value of the Master Motor to the Dashboard
    SmartDashboard.putString("Wrist Target Position", "STOW");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // wristSubsystem.stopMotor(); //Can't stop motor if you want it to get to position
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
