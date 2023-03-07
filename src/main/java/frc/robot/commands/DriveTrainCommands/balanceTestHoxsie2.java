// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class balanceTestHoxsie2 extends CommandBase {
  DrivetrainSubsystem driveTrain;
  // Our deadzones for the pitch and roll
  double pitchDeadzone = 3; // Degrees
  double rollDeadzone = 3; // degrees
  // Variables for our current pitch and roll.
  double pitchAngleDegrees;
  double rollAngleDegrees;
  // Drive speeds
  double xAxisRate;
  double yAxisRate;

  /** Creates a new balanceTestHoxsie2. */
  public balanceTestHoxsie2(DrivetrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get our current pitch and roll degrees
    pitchAngleDegrees = driveTrain.getPitch();
    rollAngleDegrees = driveTrain.getRoll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get our latest roll and pitch
    pitchAngleDegrees = driveTrain.getPitch();
    rollAngleDegrees = driveTrain.getRoll();

    // Calculate the y and x axis rates
    // if our pitch is greater than our deadzone
    if (pitchDeadzone <= Math.abs(pitchAngleDegrees)) {
      // Setup a x-axis rate
      xAxisRate = Math.sin(Math.toRadians(pitchAngleDegrees)) * -1;
    } else {
      xAxisRate = 0;
    }

    // if our roll is greater than our deadzone
    if (rollDeadzone <= Math.abs(rollAngleDegrees)) {
      // Setup a y-axis rate
      yAxisRate = Math.sin(Math.toRadians(rollAngleDegrees)) * -1;
    } else {
      yAxisRate = 0;
    }

    //  Drive forward using our xAxis and yAxis rates
    driveTrain.drive(new ChassisSpeeds(xAxisRate, yAxisRate, 0));
    // Let the motor update
    Timer.delay(0.005);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
