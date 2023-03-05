// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLightCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class GetLocationOfAprilTag extends CommandBase {
  LimelightSubsystem limeLight;
  int pipeLine;
  double [] botpose;

  /** Creates a new GetLocationOfAprilTag. */
  public GetLocationOfAprilTag(LimelightSubsystem limeLight, int pipeLine) {
    this.limeLight = limeLight;
    this.pipeLine = pipeLine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( ) {
    limeLight.setPipeline(pipeLine);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((pipeLine >= 1) && (pipeLine <=4)){
      botpose = limeLight.getBotpose_WpiRed();
    }
    else if (pipeLine == 0){
      botpose = limeLight.getBotpose_WpiBlue();
    }
    else{
      botpose = limeLight.getBotpose_WpiBlue();
    }

    SmartDashboard.putNumber("BotPose X", botpose[0]);
    SmartDashboard.putNumber("BotPose Y", botpose[1]);
    SmartDashboard.putNumber("BotPose Z", botpose[2]);
    SmartDashboard.putNumber("BotPose Roll", botpose[3]);
    SmartDashboard.putNumber("BotPose Pitch", botpose[4]);
    SmartDashboard.putNumber("BotPose Yaw", botpose[5]);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
