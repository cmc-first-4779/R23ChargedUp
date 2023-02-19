// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WinchCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class WinchTESTCommand extends CommandBase {
  WinchSubsystem winchSubsystem;
  /** Creates a new WinchTESTommand. */
  public WinchTESTCommand(WinchSubsystem winchSubsystem) {
    this.winchSubsystem = winchSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(winchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double position = SmartDashboard.getNumber("Position", 10000);
    double kF = SmartDashboard.getNumber("kF", 0.00);
    double kP = SmartDashboard.getNumber("kP", 0.05);
    double kI = SmartDashboard.getNumber("kI", 0.00);
    double kD = SmartDashboard.getNumber("kD", 0.00);
    double cruiseVel = SmartDashboard.getNumber("Cruise Vel", 8000);
    double cruiseAccel = SmartDashboard.getNumber("Cruise Accel", 8000);
    System.out.println("Inside init of Test Command.  Accel is:  "  + cruiseAccel);
    //  Call Motion Magic to set our Winch Position to the Ground
    winchSubsystem.testWinchMM(position, kF, kP, kI, kD, cruiseVel, cruiseAccel);

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
