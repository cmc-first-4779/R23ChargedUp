// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.StaticConstants.LimelightConstants;
import frc.robot.pipelines.LimelightPipelines;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTargetDeploy extends CommandBase {
  // Declare our Subsystems
  DrivetrainSubsystem drivetrain;
  LimelightSubsystem limelight;
  // Declare our drive variables
  double xDrive = 0;
  double yDrive = 0;
  double pDistance = 1.0;

  // Putting a counter so we end
  int counter;
  int maxCycles = 45;

  //  Which mode are we in
  String mode;

  /** Creates a new LimelightTargetConeDeploy. */
  public LimelightTargetDeploy(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight, String mode) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mode == "TELEOP_CONE"){
      limelight.setPipeline(LimelightPipelines.LIMELIGHT_PIPELINE_TELEOP_CONE_DEPLOY);
      limelight.setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_ON);
      limelight.setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_VISION);
    }
    else if (mode == "AUTON_CONE"){
      limelight.setPipeline(LimelightPipelines.LIMELIGHT_PIPELINE_AUTON_CONE_DEPLOY);
      limelight.setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_PIPELINE_DEFAULT);
      limelight.setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_VISION);
    }
    else if (mode == "CUBE"){
      limelight.setPipeline(LimelightPipelines.LIMELIGHT_PIPELINE_CUBE_DEPLOY);
      limelight.setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_PIPELINE_DEFAULT);
      limelight.setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_VISION);
    }
    else{  //  Double Human Player Station
      limelight.setPipeline(LimelightPipelines.LIMELIGHT_HUMAN_PLAYER_STATION);
      limelight.setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_PIPELINE_DEFAULT);
      limelight.setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_VISION);
    }
    //Set our counter to zero
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Check to make sure we have a target on the Limelight
    boolean hasTarget = limelight.hasTarget();

    //  If we have a target, calculate or drives on the x and y axes
    if (hasTarget) {
      //Calculate how much we have to move in the x direction
      xDrive = calculateXDrive();
      //Calculate how much we have to move in the y direction
      yDrive = calculateYDrive();
      // Increment our counter
      counter++;
    }

    // Drive forward using our xAxis and yAxis rates
    System.out.println("Xdrive" + xDrive);
    System.out.println("Ydrive" + yDrive*-1);
    drivetrain.drive(new ChassisSpeeds(xDrive, yDrive, 0));
    // Let the motor update
    Timer.delay(0.005);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //  Set our LED Mode to OFF
    //limelight.setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_OFF);
    //  Set our Camera Mode to driver
    //limelight.setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_DRIVER);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //  Finish until we have cycled through enough times
    return counter >= maxCycles;
  }

  //  Use our TY Error to control the x-position of the swerve.
  //    Having the x and y swapped here is not an error..  
  //    The Limelight's axis is different from field orientation for swerve
  private double calculateXDrive() {
    double xError = limelight.getTY();
    if (xError > Constants.LIMELIGHT_X_DRIVE_TOLERANCE) {
      xDrive = (pDistance * xError) + Constants.LIMELIGHT_MIN_MOVE;
    } else if (xError < -Constants.LIMELIGHT_X_DRIVE_TOLERANCE) {
      xDrive = (pDistance * xError) - Constants.LIMELIGHT_MIN_MOVE;
    } else {
      xDrive = 0;
    }
    
    return xDrive ;
  }

    //  Use our TX Error to control the y-position of the swerve.
  //    Having the x and y swapped here is not an error..  
  //    The Limelight's axis is different from field orientation for swerve
  private double calculateYDrive() {
    double yError = limelight.getTX();
    if (yError > Constants.LIMELIGHT_Y_DRIVE_TOLERANCE) {
      yDrive = (pDistance * yError) + Constants.LIMELIGHT_MIN_MOVE;
    } else if (yError < -Constants.LIMELIGHT_Y_DRIVE_TOLERANCE) {
      yDrive = (pDistance * yError) - Constants.LIMELIGHT_MIN_MOVE;
    } else {
      yDrive = 0;
    }
    
    return yDrive * -1;
  }

}
