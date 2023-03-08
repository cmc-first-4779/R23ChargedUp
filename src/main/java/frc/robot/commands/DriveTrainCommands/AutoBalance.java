// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

//  This command autobalances the robot on the charging station.  It takes 
//   the pitch and yaw off of the gyro and feeds it into our swerve drive to 
//   guide the robot onto the platform and level out.

public class AutoBalance extends CommandBase {
  DrivetrainSubsystem driveTrain;
  // Our deadzones for the pitch and roll
  double pitchDeadzone = Constants.DRIVETRAIN_AUTOBALANCE_DEADZONE_Y; // Degrees
  double rollDeadzone = Constants.DRIVETRAIN_AUTOBALANCE_DEADZONE_X;; // degrees
  // Variables for our current pitch and roll (in degrees)
  double pitchAngleDegrees;
  double rollAngleDegrees;
  // Drive speeds
  double xAxisRate;
  double yAxisRate;

  /** Creates a new AutoBalance */
  public AutoBalance(DrivetrainSubsystem driveTrain) {
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

    // If the pitch angle is >= our full pitch deadzone
    if (Math.abs(pitchAngleDegrees) >= pitchDeadzone) {
      // then go full throttle to correct
      yAxisRate = Math.sin(Math.toRadians(pitchAngleDegrees)) * (-1) * Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE;
    }
    // Else if our pitch angle is half of our pitch deadzone
    else if (Math.abs(pitchAngleDegrees) >= (pitchDeadzone / 2)) {
      // then go half throttle to correct
      yAxisRate = Math.sin(Math.toRadians(pitchAngleDegrees)) * (-1) * (Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE / 2);
    }
    // Else if our pitch angle is half of our pitch deadzone
    else if (Math.abs(pitchAngleDegrees) >= (pitchDeadzone / 4)) {
      // then go quarter throttle to correct
      yAxisRate = Math.sin(Math.toRadians(pitchAngleDegrees)) * (-1) * (Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE / 4);
    }
    // Else if our pitch angle is eigth of our pitch deadzone
    else if (Math.abs(pitchAngleDegrees) >= (pitchDeadzone / 8)) {
      // then go quarter throttle to correct
      yAxisRate = Math.sin(Math.toRadians(pitchAngleDegrees)) * (-1) * (Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE / 8);
    } 
    //  Let's not go anywhere on the y-axis
    else {
      yAxisRate = 0;
    }

    // Calculate the y and x axis rates
    // if our pitch is greater than our deadzone band
    // if (pitchDeadzone <= Math.abs(pitchAngleDegrees)) {
    // // Setup a y-axis rate
    // yAxisRate = Math.sin(Math.toRadians(pitchAngleDegrees)) * (-1) *
    // Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE;
    // } else {
    // yAxisRate = 0;
    // }

    // If the roll angle is >= our full pitch deadzone
    if (Math.abs(rollAngleDegrees) >= rollDeadzone) {
      // then go full throttle to correct
      xAxisRate = Math.sin(Math.toRadians(rollAngleDegrees)) * (-1) * Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE;
    }
    // Else if our roll angle is half of our pitch deadzone
    else if (Math.abs(rollAngleDegrees) >= (rollDeadzone / 2)) {
      // then go half throttle to correct
      xAxisRate = Math.sin(Math.toRadians(rollAngleDegrees)) * (-1) * (Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE / 2);
    }
    // Else if our roll angle is half of our pitch deadzone
    else if (Math.abs(rollAngleDegrees) >= (rollDeadzone / 4)) {
      // then go quarter throttle to correct
      xAxisRate = Math.sin(Math.toRadians(rollAngleDegrees)) * (-1) * (Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE / 4);
    }
    // Else if our pitch angle is eigth of our pitch deadzone
    else if (Math.abs(rollAngleDegrees) >= (rollDeadzone / 8)) {
      // then go quarter throttle to correct
      xAxisRate = Math.sin(Math.toRadians(rollAngleDegrees)) * (-1) * (Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE / 8);
    } 
    //  Else let's not go anywhere on the x-axis
    else {
      xAxisRate = 0;
    }

    // // if our roll is greater than our deadzone band
    // if (rollDeadzone <= Math.abs(rollAngleDegrees)) {
    // // Setup a x-axis rate
    // xAxisRate = Math.sin(Math.toRadians(rollAngleDegrees)) * (-1) *
    // Constants.DRIVETRAIN_AUTOBALANCE_THROTTLE;
    // } else {
    // xAxisRate = 0;
    // }

    // Drive forward using our xAxis and yAxis rates
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
