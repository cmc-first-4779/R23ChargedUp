// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShoulderCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderTESTCommand extends CommandBase {
  ShoulderSubsystem shoulder;
  /** Creates a new WinchTESTommand. */
  public ShoulderTESTCommand(ShoulderSubsystem shoulder) {
    this.shoulder = shoulder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double position = SmartDashboard.getNumber("Position", 10000);
    //  Call Motion Magic to set our Winch Position to the Ground
    shoulder.setShoulderPosition(position);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Put the encoder value of the Master Motor to the Dashboard
        SmartDashboard.putString("Winch Target Position", "TEST");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // winchSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
