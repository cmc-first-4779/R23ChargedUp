// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

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
