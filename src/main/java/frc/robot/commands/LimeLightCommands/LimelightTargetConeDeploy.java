// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StaticConstants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTargetConeDeploy extends CommandBase {
  LimelightSubsystem limelight;
  /** Creates a new LimelightTargetConeDeploy. */
  public LimelightTargetConeDeploy(LimelightSubsystem limelight) {
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_ON);
    limelight.setPipeline(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
