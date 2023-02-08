// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WristSubsystem;

public class WristLowConeCommand extends CommandBase {
  // Declare our subsystem
  WristSubsystem wristSubsystem;

  /** Creates a new WristStopCommand. */
  public WristLowConeCommand(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.setWristPosition(Constants.WRIST_POSITION_LOW_CONE_NODE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Put the encoder value of the Master Motor to the Dashboard
    SmartDashboard.putString("Wrist Target Position", "LOW CONE");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}