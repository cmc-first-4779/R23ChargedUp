// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WristSubsystem;

public class WristTESTCommand extends CommandBase {
   // Declare our subsystem
   WristSubsystem wristSubsystem;
  /** Creates a new WristTESTCommand. */
  public WristTESTCommand(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double position = SmartDashboard.getNumber("Position", 0);
    double kF = SmartDashboard.getNumber("kF", Constants.WRIST_kF);
    double kP = SmartDashboard.getNumber("kP", Constants.WRIST_kP);
    double kI = SmartDashboard.getNumber("kI", Constants.WRIST_kI);
    double kD = SmartDashboard.getNumber("kD", Constants.WRIST_kD);
    double maxVel = SmartDashboard.getNumber("Max Vel", Constants.WRIST_SM_MAX_VEL);
    double minVel = SmartDashboard.getNumber("Min Vel", Constants.WRIST_SM_MIN_VEL);
    double maxAccel = SmartDashboard.getNumber("Max Accel", Constants.WRIST_SM_MAX_ACCEL);
    wristSubsystem.testWristPosition(position , kF, kP, kI, kD, 1, -1, maxVel, minVel, maxAccel, Constants.WRIST_SM_ALLOWED_ERR, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Put the encoder value of the Master Motor to the Dashboard  -- Already happenign in periodic() of wrist subsystem
        // SmartDashboard.putString("Wrist Target Position", "TEST");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // wristSubsystem.stopMotor(); //Can't have it stop motor since we need it to allow the motor to get to the position.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // Returning true will end the command
  }
}
