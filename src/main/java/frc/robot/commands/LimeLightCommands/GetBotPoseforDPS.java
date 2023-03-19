// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightCommands;

import java.nio.channels.Pipe;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StaticConstants.AprilTags;
import frc.robot.StaticConstants.Pipelines;
import frc.robot.subsystems.LimelightSubsystem;


//  This Command returns the BotPose in regards to the 
//   Double Human Player Station
//    April Tag #4 if we are the Red Alliance
//    April Tag #5 if we are the Blue Alliance

public class GetBotPoseforDPS extends CommandBase {

  LimelightSubsystem limeLight;
  int pipeline;
  int aprilTagNumber;
  double [] botpose;
  public DriverStation.Alliance allianceColor;
  
  /** Creates a new GetBotPoseforDPS. */
  public GetBotPoseforDPS(LimelightSubsystem limeLight) {
    this.limeLight = limeLight;
    //  Get our Alliance Color from the Driverstation
    allianceColor = DriverStation.getAlliance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Set our April Tag # based on our Alliance Color
    if (allianceColor == Alliance.Red){
      aprilTagNumber = AprilTags.RED_DOUBLE_HUMAN_PLAYER_STATION;
      pipeline = Pipelines.RED_DOUBLE_HUMAN_PLAYER_STATION;
      limeLight.setPipeline(pipeline);
    }
    else{
      aprilTagNumber = AprilTags.RED_DOUBLE_HUMAN_PLAYER_STATION;
      pipeline = Pipelines.BLUE_DOUBLE_HUMAN_PLAYER_STATION;
      limeLight.setPipeline(pipeline);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (allianceColor == Alliance.Red){
      botpose = limeLight.getBotpose_WpiRed();
      SmartDashboard.putString("Alliance", "RED");
    }
    else{
      botpose = limeLight.getBotpose_WpiBlue();
      SmartDashboard.putString("Alliance", "BLUE");
    }
    SmartDashboard.putNumber("BotPoseX", botpose[0]);
    SmartDashboard.putNumber("BotPoseY", botpose[1]);
    SmartDashboard.putNumber("BotPoseZ", botpose[2]);
    SmartDashboard.putNumber("BotPoseRoll", botpose[3]);
    SmartDashboard.putNumber("BotPosePitch", botpose[4]);
    SmartDashboard.putNumber("BotPoseYaw", botpose[5]);
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
