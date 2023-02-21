// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.StaticConstants.LimelightConstants;
import frc.robot.Constants;
import frc.robot.commands.SetPipeline;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//  Our Limelight Subsystem is where all of our Vision Processing Takes place

public class LimelightSubsystem extends SubsystemBase {


  // Declare the Network Table that our Limelight will broadcast its values on
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // Declare the table entries for tx, ty, ta, and ts
  NetworkTableEntry tx = table.getEntry("tx"); // how far "x" axis is from our target
  NetworkTableEntry ty = table.getEntry("ty"); // how far "y" axis is from our target
  NetworkTableEntry ta = table.getEntry("ta"); // how much total area is the target taking up. (how far away)
  NetworkTableEntry ts = table.getEntry("ts"); // How far are we skewed fromt the target

  // read values periodically
  double x = tx.getDouble(0.000);
  double y = ty.getDouble(0.000);
  double area = ta.getDouble(0.0);
  double skew = ts.getDouble(0.0);

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }

  // SET THE LIMELIGHT CAMERA MODE
  // 0 = Limelight Vision. VISION PROCESSING
  // 1 = Driver Camera (Increases exposure, disables vision processing)
  public void setCameraMode(double camMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
  }

  // TURN THE LIMELIGHT LED LIGHTS ON AND OFF
  // 0 = use the LED Mode set in the current pipeline
  // 1 = Force the LEDs OFF
  // 2 = Force the LEDs to BLINK
  // 3 = Force the LEDs ON
  public void setLEDMode(double ledMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode);
  }

  // SET WHICH VISION PIPELINE WE WANT TO USE..
  // Pipelines: 0 - 9
  // Each pipeline can be configured for a specific vision target
  public void setPipeline(double pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }

  // Return TX, which is the Horizontal Offset From Crosshair To Target (-27
  // degrees to 27 degrees)
  public double getTX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  // Returns TY, which is the Vertical Offset From Crosshair To Target (-20.5
  // degrees to 20.5 degrees)
  public double getTY() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  // Returns TV, which is whether the limelight has any valid targets
  // 0 = NO TARGET
  // 1 = TARGET
  public double getTV() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }

  // Returns TA, which is the Target Area (0% of image to 100% of image)
  public double getTA() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  // Returns TS, which is the Skew or rotation (-90 degrees to 0 degrees)
  public double getTS() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
  }

  // Returns whether or not the Limelight has a target...
  public boolean hasTarget() {
    double tv = getTV();
    if (tv == LimelightConstants.LIMELIGHT_NO_TARGET) {
      return false;
    } else {
      return true;
    }
  }

  // Set the Streaming Mode on the Limelight
  public void setStreamingMode(double streamingMode) {
    double m_streaming_mode = streamingMode;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(m_streaming_mode);
    setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_ON);
  }

  // Set the Snapshot mode. Allows users to take snapshots during a match
  // 0 ... Resets snapshot mode
  // 1 ... Take exactly one snapshot
  public void setSnapshotMode(double snapshotMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(snapshotMode);
    setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_ON);
  }

  // Method that returns whether the Limelight crosshairs are "head on"
  public boolean isHeadonTarget() {
    return getTS() == 0;
  }

  // Method that returns whether our crosshairs are right of the target
  public boolean isRightOfTarget() {
    double ts = getTS();
    return ts <= Constants.LIMELIGHT_SKEW_CLOCKWISE_MAX && ts >= Constants.LIMELIGHT_SKEW_CLOCKWISE_MIN;
  }

  // Method that returns whether our crosshairs are left of the target
  public boolean isLeftOfTarget() {
    double ts = getTS();
    return ts >= Constants.LIMELIGHT_SKEW_COUNTERCLOCKWISE_MAX && ts <= Constants.LIMELIGHT_SKEW_COUNTERCLOCKWISE_MIN;
  }

  // Setup our Initial Configuration settings for our Limelight. As the match goes
  // on, these may change
  public void initLimelightforVision() {
    // Set the green LED On
    setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_ON);
    // Set the Camera mode to "Vision" so that the Vision can use it
    setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_VISION);
  }

  // Adjust the Limelight settings for the driver mode
  public void initLimelightforDriver() {
    // Set the green LED Off
    setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_OFF);
    // Set the Camera mode to "Driver" so that the Driver can use it
    setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_DRIVER);
  }

  // APRIL TAG METHODS!!!

  // Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)
  public double[] getBotpose() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
  }

  // Robot transform in field-space (blue driverstation WPILIB origin).
  // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)
  public double[] getBotpose_WpiBlue() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);
  }

  // Robot transform in field-space (blue driverstation WPILIB origin).
  // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)
  public double[] getBotpose_WpiRed() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired")
        .getDoubleArray(new double[6]);
  }

  // 3D transform of the camera in the coordinate system of the primary in-view
  // AprilTag (array (6))
  public double[] getCamerapose_targetspace() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
        .getDoubleArray(new double[6]);
  }

  // 3D transform of the primary in-view AprilTag in the coordinate system of the
  // Camera (array (6))
  public double[] getTargetpose_cameraspace() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
        .getDoubleArray(new double[6]);
  }

  // 3D transform of the primary in-view AprilTag in the coordinate system of the
  // Robot (array (6))
  public double[] getTargetpose_robotspace() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace")
        .getDoubleArray(new double[6]);
  }

  // 3D transform of the robot in the coordinate system of the primary in-view
  // AprilTag (array (6))
  public double[] getBotpose_targetspace() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace")
        .getDoubleArray(new double[6]);
  }

  // ID of the primary in-view AprilTag
  public double[] getTid() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid")
        .getDoubleArray(new double[6]);
  }

  public void setCropValues() {
    // Declare an array to store our crop values
    double[] cropValues = new double[4];
    // Populate the array with our Constant Values
    cropValues[0] = Constants.LIMELIGHT_CROP_X0; // X0
    cropValues[1] = Constants.LIMELIGHT_CROP_X1; // X1
    cropValues[2] = Constants.LIMELIGHT_CROP_Y0; // Y0
    cropValues[3] = Constants.LIMELIGHT_CROP_Y1; // Y1
    // Push the array back ot the Limelight Network Table to set it.
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(cropValues);
  }
  

  
}
