// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.StaticConstants;

/**
 * These are values from the Limelight Network API table...
 * These NEVER change and so we are leaving them in a seperate class so that
 * RobotMap is cleaner and only contains values that we would be changing to
 * tune the robot..
 * 
 * We will leave the changing Limelight values STILL in RobotMap...
 */

public class LimelightConstants {
    //  LIMELIGHT Camera Modes
    public static final double LIMELIGHT_CAMMODE_VISION = 0; // Vision Processing Mode = 0
    public static final double LIMELIGHT_CAMMODE_DRIVER = 1; // Driver Mode = 1
    //  LIMELIGHT LED Modes
    public static final double LIMELIGHT_LEDMODE_PIPELINE_DEFAULT = 0;  // Uses the Default LED Mode for the Pipeline
    public static final double LIMELIGHT_LEDMODE_OFF = 1; // Force LED Mode Off
    public static final double LIMELIGHT_LEDMODE_BLINK = 2; // Force LED Mode Blink
    public static final double LIMELIGHT_LEDMODE_ON = 3; // Force LED Mode On
     // LIMELIGHT TV Value when there is no target
     public static final double LIMELIGHT_NO_TARGET = 0.0; // TV Value when the Limelight doesn't see its target
     public static final double LIMELIGHT_HAS_TARGET = 1.0;  //TV Value when the Limelight has a target
     //  Camera Pixel Count and Field of View Constants
     public static final double LIMELIGHT_X_PIXEL_COUNT = 320; // pixels
     public static final double LIMELIGHT_Y_PIXEL_COUNT = 240; // pixels
     public static final double LIMELIGHT_X_FOV = 59.6; // degrees
     public static final double LIMELIGHT_Y_FOV = 45.7; // degrees
  
     //  Limelight Streaming Modes
     public static final double LIMELIGHT_STREAMING_STANDARD_MODE = 0;  //  Side-by-side streams if a webcam is attached
     public static final double LIMELIGHT_PiP_MAIN_MODE = 1;  //Secondary camera stream is placed in the lower right camera
     public static final double LIMELIGHT_PiP_SECONDARY_MODE = 2;  //primary camera is placed in lower right of secondary
  // Limelight Constants
  public static final double LIMELIGHT_PIPELINE_PORT_CLOSE = 0; // Use the Upper Power Port Pipeline
  public static final double LIMELIGHT_PIPELINE_PORT_FAR = 1; // from > 14 ft away
  public static final double LIMELIGHT_AIMING_DEADBAND = 1; // was 1.75 // How close do we have to be right or left of
                                                            // the target
  public static final double LIMELIGHT_AIMING_DISTANCE_TOLERANCE = 2; // Tolerance of degrees we want to be on target
  public static final double LIMELIGHT_AIMING_kpAim = 0.0040;
  public static final double LIMELIGHT_AIMING_kpDist = 0.05;
  public static final double LIMELIGHT_AIMING_AIM_MIN_CMD = 0.113; // adjust this for turn speed once we found a target
  public static final double LIMELIGHT_AIMING_MOVE_MIN_CMD = 0.26;
  public static final int LIMELIGHT_AIMING_COUNTER = 10; // Number of times we want to aim before taking a shot

  // LIMELIGHT SEEK MODE - How much power do we give the motors when it is turning
  // to scan and driving...
  public static final double LIMELIGHT_SEEK_TURN_DT_POWER = 0.45; // Power when we are turning when we CAN'T SEE THE
                                                                  // target
  public static final double LIMELIGHT_SEEK_DRIVE_DT_POWER = 0.7; // Power when we are driving toward the target
  public static final double LIMELIGHT_SEEK_TURN_TURRET_POWER = 0.4; // Power when we are using the turret to seek

  // Tolerance of degrees we want to be on target
  public static final double LIMELIGHT_SKEW_CLOCKWISE_MAX = 62.5; // degrees
  public static final double LIMELIGHT_SKEW_CLOCKWISE_MIN = -90.0; // degrees
  public static final double LIMELIGHT_SKEW_COUNTERCLOCKWISE_MAX = -0.01; // degrees
  public static final double LIMELIGHT_SKEW_COUNTERCLOCKWISE_MIN = 32.5; // degrees

  // Limelight Crop Values
  public static final double LIMELIGHT_CROP_X0 = -1.0;
  public static final double LIMELIGHT_CROP_X1 = 1.0;
  public static final double LIMELIGHT_CROP_Y0 = -1.0;
  public static final double LIMELIGHT_CROP_Y1 = 1.0;



}
