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

  // Extender Arm Constants
  public static final int EXTENDER_PID_SLOT = 0;
  public static final double EXTENDER_kF = 0.0; // Dummy variable - Test this!
  public static final double EXTENDER_kP = 0.0005; // Dummy variable - Test this!
  public static final double EXTENDER_kI = 0.0; // Dummy variable - Test this!
  public static final double EXTENDER_kD = 0.0; // Dummy variable - Test this!
  public static final double EXTENDER_kMaxOuput = 1;
  public static final double EXTENDER_kMinOutput = -1;
  public static final double EXTENDER_MAX_RPM = 5700;
  public static final double EXTENDER_POSITION_STOW = 0; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_GROUND = 0; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_LOW_CONE_NODE = 20; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_MID_CONE_NODE = 40; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_HIGH_CONE_NODE = 80; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_LOW_CUBE_NODE = 20; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_MID_CUBE_NODE = 40; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_HIGH_CUBE_NODE = 80; // Dummy variable - Test this!
  public static final double EXTENDER_SM_MAX_VEL = 2000; // Dummy variable - Test this!
  public static final double EXTENDER_SM_MIN_VEL = 0; // Dummy variable - Test this!
  public static final double EXTENDER_SM_MAX_ACCEL = 1500; // Dummy variable - Test this!
  public static final double EXTENDER_SM_ALLOWED_ERR = .25; // Dummy variable - Test this!
  public static final double EXTENDER_MAX_POSTION = 100; // Max forward rotation. Current gearing is xx to one so position of xx is one full rotation of output shaft.  Only need to go about 25%
  public static final double EXTENDER_MIN_POSTION = 0; // Min forward rotation. Assuming we are starting in our minimal position of 0.
  public static final double EXTENDER_MOVEMENT_INCREMENT = 2;
  public static final int EXTENDER_MINIMUM_ARM_POSITION_TO_EXTEND = 0;

}
