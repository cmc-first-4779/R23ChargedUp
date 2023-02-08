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
  public static final double WINCH_MM_VELOCITY = 5000;  // Dummy variable - Test this!
  public static final double WINCH_MM_ACCELERATION = 2500; // Dummy variable - Test this!
  public static final double WINCH_NOMINAL_OUTPUT_FORWARD = 0.0; 
  public static final double WINCH_NOMINAL_OUTPUT_REVERSE = 0.0; 
  public static final double WINCH_PEAK_OUTPUT_FORWARD = 1.0; 
  public static final double WINCH_PEAK_OUTPUT_REVERSE = -1.0; 
  public static final double WINCH_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS = 0.5; // Dummy variable - Test this!
  public static final double WINCH_NEUTRAL_DEADBAND_PERCENT = 0.04;  // 0.04 is the default
  public static final double WINCH_DEFAULT_P = 0.12;// Dummy variable - Test this!
  public static final double WINCH_DEFAULT_I = 0.00;// Dummy variable - Test this!
  public static final double WINCH_DEFAULT_D = 0.01;// Dummy variable - Test this!
  public static final double WINCH_DEFAULT_F = 0.52;// Dummy variable - Test this!
  public static final double WINCH_POSITION_STOW = 0;  // Dummy variable - Test this!
  public static final double WINCH_POSITION_GROUND = 2000;   // Dummy variable - Test this!
  public static final double WINCH_POSITION_LOW_CONE_NODE = 4000;  // Dummy variable - Test this!
  public static final double WINCH_POSITION_MID_CONE_NODE = 8000;  // Dummy variable - Test this!
  public static final double WINCH_POSITION_HIGH_CONE_NODE = 12000;  // Dummy variable - Test this!
  public static final double WINCH_POSITION_LOW_CUBE_NODE = 4000;  // Dummy variable - Test this!
  public static final double WINCH_POSITION_MID_CUBE_NODE = 8000;  // Dummy variable - Test this!
  public static final double WINCH_POSITION_HIGH_CUBE_NODE = 12000;  // Dummy variable - Test this!

}
