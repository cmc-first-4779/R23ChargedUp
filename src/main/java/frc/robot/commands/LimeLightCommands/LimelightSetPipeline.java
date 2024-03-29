// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightSetPipeline extends CommandBase {
  boolean isDone = false;
  LimelightSubsystem myLimelight;
  int pipeline;


  /** Creates a new SetPipeline. */
  public LimelightSetPipeline(LimelightSubsystem limelight, int pipeline) {
    myLimelight = limelight;
    this.pipeline = pipeline;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myLimelight.setPipeline(pipeline);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("GetPipeline", pipeline);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
