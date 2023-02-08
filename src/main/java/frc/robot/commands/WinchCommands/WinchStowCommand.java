// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WinchCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WinchSubsystem;

public class WinchStowCommand extends CommandBase {
  WinchSubsystem winchSubsystem;
  /** Creates a new WinchGroundCommand. */
  public WinchStowCommand(WinchSubsystem winchSubsystem) {
    this.winchSubsystem = winchSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(winchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Call Motion Magic to set our Winch Position to the Ground
    winchSubsystem.setWinchPositionMM(Constants.WINCH_POSITION_STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Put the encoder value of the Master Motor to the Dashboard
        SmartDashboard.putString("Winch Target Position", "STOW");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    winchSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
