// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // CTR PID Constants
  public static final int PID_CLOSED_LOOP = 0;
  public static final int PID_AUX_LOOP = 1;
  public static final int kTimeoutMs = 30;

  //  WINCH Constants
  public static final double SHOULDER_MM_VELOCITY = 8000;  // Dummy variable - Test this!
  public static final double SHOULDER_MM_ACCELERATION = 8000; // Dummy variable - Test this!
  public static final double SHOULDER_NOMINAL_OUTPUT_FORWARD = 0.0; 
  public static final double SHOULDER_NOMINAL_OUTPUT_REVERSE = 0.0; 
  public static final double SHOULDER_PEAK_OUTPUT_FORWARD = 1.0; 
  public static final double SHOULDER_PEAK_OUTPUT_REVERSE = -1.0; 
  public static final double SHOULDER_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS = .5; // Dummy variable - Test this!
  public static final double SHOULDER_NEUTRAL_DEADBAND_PERCENT = 0.04;  // 0.04 is the default
  public static final double SHOULDER_ALLOWED_ERROR = 250;
  public static final double SHOULDER_DEFAULT_kP = 0.05;// Dummy variable - Test this!
  public static final double SHOULDER_DEFAULT_kI = 0.00;// Dummy variable - Test this!
  public static final double SHOULDER_DEFAULT_kD = 0.00;// Dummy variable - Test this!
  public static final double SHOULDER_DEFAULT_kF = 0.00;// Dummy variable - Test this!
  public static final double SHOULDER_MAX_GRAVITY_kF = 0.07;
  public static final double SHOULDER_POSITION_MIN = 0;  //  Min position of the arm
  public static final double SHOULDER_POSITION_SAFE_TO_EXTEND = 40000;  //  Safe distance where can extend the rest of the arm
  public static final double SHOULDER_POSITION_MAX = 90000;  // Max position of the arm
  public static final double SHOULDER_POSITION_STOW = 0;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_GROUND = 5000;   // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_LOW_CONE_NODE = 10000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_MID_CONE_NODE = 25000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_HIGH_CONE_NODE = 70000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_LOW_CUBE_NODE = 15000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_MID_CUBE_NODE = 30000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_HIGH_CUBE_NODE = 75000;  // Dummy variable - Test this!
  public static final double SHOULDER_MOVEMENT_INCREMENT = 2500;
}
