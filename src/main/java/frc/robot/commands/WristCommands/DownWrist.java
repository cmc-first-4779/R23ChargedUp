// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WristSubsystem;

public class DownWrist extends CommandBase {
  WristSubsystem wrist;
  double rightPosition;
  double leftPosition;
  boolean isItFinished;

  /** Creates a new DownCommand. */
  public DownWrist(WristSubsystem wrist) {
    this.wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightPosition = wrist.getRightEncoderPosition();
    leftPosition = wrist.getLeftEncoderPosition();
    isItFinished = false;
    wrist.setPosition(Constants.WRIST_DOWN_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightPosition = wrist.getRightEncoderPosition();
    leftPosition = wrist.getLeftEncoderPosition();
    SmartDashboard.putNumber("Right Wrist Position", rightPosition);
    SmartDashboard.putNumber("Left Wrist Position", leftPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rightPosition <= Constants.WRIST_DOWN_POSITION) {
      isItFinished = true;
    }
    if (leftPosition <= Constants.WRIST_DOWN_POSITION) {
      isItFinished = true;
    }

    return false;
  }
}
