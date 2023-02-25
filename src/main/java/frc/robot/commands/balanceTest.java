// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class balanceTest extends CommandBase {
  DrivetrainSubsystem m_drive;
  double m_pigeonPitch;

  /** Creates a new balanceTest. */
  public balanceTest(DrivetrainSubsystem drive) {
   m_drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pigeonPitch = m_drive.getPitch();
    if (m_pigeonPitch <= Constants.PIGEON_DEADBAND_GOING_DOWN) {
      m_drive.driveBackwardsSlow();
    }
    if (m_pigeonPitch >= Constants.PIGEON_DEADBAND_GOING_UP) {
      m_drive.driveStraightSlow();
    }
    else {
      m_drive.sturdyBase();
      
    }
    // if (m_pigeonPitch <= Constants.PIGEON_DEADBAND_GOING_DOWN) {
    //   m_drive.driveBackwardsSlow();
    // }
    // else {
    //   m_drive.sturdyBase();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
