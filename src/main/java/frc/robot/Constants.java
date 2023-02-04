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

  public static final double GRIPPER_ENCODER_MAX = 4;

  public static final double GRIPPER_ENCODER_MIN = 0;

  public static final double EXTENDER_SPEED = 1;
  public static final double EXTENDER_LOW_POSITION = 5; // dummy VARIABLE
  public static final double EXTENDER_MED_POSITION = 10; // DUMMY VARIABLE
  public static final double EXTENDER_HIGH_POSITION = 15; // DUMMY VARIABLE
  public static final double EXTENDER_START_POSITION = 0;// DUMMY VARIABLE

  public static final double PIVOT_SPEED = 1; // DUMMY VARIABLE
  public static final double PIVOT_START_POSITION = 0; // DUMMY VARIABLE
  public static final double PIVOT_LOW_POSITION = 5; // DUMMY VARIABLE
  public static final double PIVOT_MID_POSITION = 10; // DUMMY VARIABLE
  public static final double PIVOT_HIGH_POSITION = 15; // DUMMY VARIABLE
  public static final double PIVOT_kF = 0.5; // DUMMY VARIABLE
  public static final double PIVOT_kP = 0.13; // DUMMY VARIABLE
  public static final double PIVOT_kI = 0.0; // DUMMY VARIABLE
  public static final double PIVOT_kD = 0.0; // DUMMY VARIABLE
  public static final double PIVOT_MM_VELOCITY = 15000;// DUMMY VARIABLE
  public static final double PIVOT_MM_ACCELERATION = 6000; // DUMMY VARIABLE
  
  public static final double WRIST_MM_ACCELERATION = 6000; // DUMMY VARIABLE
  public static final double WRIST_MM_VELOCITY = 15000; // DUMMY VARIABLE
}
