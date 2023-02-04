// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WristSubsystem;

public class upWrist extends CommandBase {
  WristSubsystem wrist;
  double rightPosition;
  double leftPosition;
  boolean isItFinished;

  /** Creates a new upWrist. */
  public upWrist(WristSubsystem wrist) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightPosition = wrist.getRightEncoderPosition();
    leftPosition = wrist.getLeftEncoderPosition();
    wrist.moveWristMM(Constants.WRIST_UP_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    wrist.stopMotors(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rightPosition >= Constants.WRIST_UP_POSITION){
      isItFinished = true;
    }
    if (leftPosition >= Constants.WRIST_UP_POSITION){
      isItFinished = true;
    }
    
    return isItFinished;


  }
}
