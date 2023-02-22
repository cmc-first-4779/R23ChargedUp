// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class sturdyBaseCommand extends CommandBase {
  DrivetrainSubsystem m_drive;

  /** Creates a new sturdyBaseCommand. */
  public sturdyBaseCommand(DrivetrainSubsystem drive) {
    m_drive = drive; 
       // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.backLeftModule.set(0, -45);
    m_drive.backRightModule.set(0, 45);
    m_drive.frontRightModule.set(0, -45);
    m_drive.frontLeftModule.set(0, 45);
    
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
