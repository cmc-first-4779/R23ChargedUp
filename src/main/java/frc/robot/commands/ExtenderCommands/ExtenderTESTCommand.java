// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtenderCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderTESTCommand extends CommandBase {
  ExtenderSubsystem extenderSubsystem;
  /** Creates a new ExtenderTESTCommand. */
  public ExtenderTESTCommand(ExtenderSubsystem extenderSubsystem) {
    this.extenderSubsystem = extenderSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double position = SmartDashboard.getNumber("Position", 40);
    extenderSubsystem.setExtenderPosition(position);
    //extenderSubsystem.testExtenderPosition(position , kF, kP, kI, kD, 1, -1, maxVel, minVel, maxAccel, Constants.EXTENDER_SM_ALLOWED_ERR, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Put the encoder value of the Master Motor to the Dashboard
        SmartDashboard.putString("Extender Target Position", "TEST");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //extenderSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
