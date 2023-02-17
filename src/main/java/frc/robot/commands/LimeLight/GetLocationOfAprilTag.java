// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class GetLocationOfAprilTag extends CommandBase {
  LimelightSubsystem limeLight;
  int pipeLine;

  /** Creates a new GetLocationOfAprilTag. */
  public GetLocationOfAprilTag(LimelightSubsystem limeLight, int pipeLine) {
    this.limeLight = limeLight;
    this.pipeLine = pipeLine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
