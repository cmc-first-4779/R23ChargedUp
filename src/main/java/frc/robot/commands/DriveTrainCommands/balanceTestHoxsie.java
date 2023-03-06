// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class balanceTestHoxsie extends CommandBase {

  private final DrivetrainSubsystem driveTrain;
  private final PIDController forwardController;

  private final double kP = 0.1;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final double maxOutput = 0.5;

  private double currentAngle;
  //private double error;
  private double output;
  
  /** Creates a new balanceTestHoxsie. */
  public balanceTestHoxsie(DrivetrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    forwardController = new PIDController(kP, kI, kD);
    forwardController.setTolerance(2.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Set our PID Setpoint to 0 for the pitch
    forwardController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = driveTrain.getPitch();
    output = MathUtil.clamp(forwardController.calculate(currentAngle, 0), maxOutput, maxOutput);

    //driveTrain.drive(new Translation2d(output, 0), 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forwardController.atSetpoint();
  }
}
