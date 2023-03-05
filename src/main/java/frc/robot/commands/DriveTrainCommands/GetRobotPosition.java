// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class GetRobotPosition extends CommandBase {
  boolean isDone = false;
  LimelightSubsystem myLimelight;
  int myPipeline;
  

  /** Creates a new SetPipeline. */
  public GetRobotPosition(LimelightSubsystem limelight, int pipeline) {
      myLimelight = limelight;
      myPipeline = pipeline;
      
      
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myLimelight.setPipeline(myPipeline);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (myLimelight.hasTarget()) {
      double[ ] robotPosition = myLimelight.getBotpose();
      SmartDashboard.putNumber("Robot X", robotPosition[0]);
      SmartDashboard.putNumber("Robot Y", robotPosition[1]);
      SmartDashboard.putNumber("Robot Z", robotPosition[2]);
      SmartDashboard.putNumber("Robot Roll", robotPosition[3]);
      SmartDashboard.putNumber("Robot Pitch", robotPosition[4]);
      SmartDashboard.putNumber("Robot Yaw", robotPosition[5]);
      isDone = true;
    } else {
      SmartDashboard.putNumber("Robot X", 0);
      SmartDashboard.putNumber("Robot Y", 0);
      SmartDashboard.putNumber("Robot Z", 0);
      SmartDashboard.putNumber("Robot Roll", 0);
      SmartDashboard.putNumber("Robot Pitch", 0);
      SmartDashboard.putNumber("Robot Yaw", 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  } 
}
