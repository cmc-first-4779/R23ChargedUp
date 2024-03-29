// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShoulderCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PositionSetpoints;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderStowCommand extends CommandBase {
  ShoulderSubsystem shoulderSubsystem;
  /** Creates a new WinchGroundCommand. */
  public ShoulderStowCommand(ShoulderSubsystem shoulderSubsystem) {
    this.shoulderSubsystem = shoulderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Call Motion Magic to set our Winch Position to the Ground
    shoulderSubsystem.setShoulderPosition(PositionSetpoints.SHOULDER_POSITION_STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Put the encoder value of the Master Motor to the Dashboard
        SmartDashboard.putString("Winch Target Position", "STOW");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulderSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
