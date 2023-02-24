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

  //  Intake Constants
  public static final double INTAKE_SPEED = 1.00;  //  Dummy value.  NEED TO TEST THIS!
  public static final double EJECT_SPEED = -1.00;  //  Dummy value.  NEED TO TEST THIS!
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
  public static final double EXTENDER_POSITION_PICKUP_CUBE = 10; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_PICKUP_CONE = 10; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_LOW_CONE_NODE = 20; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_MID_CONE_NODE = 40; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_HIGH_CONE_NODE = 80; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_LOW_CUBE_NODE = 20; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_MID_CUBE_NODE = 40; // Dummy variable - Test this!
  public static final double EXTENDER_POSITION_HIGH_CUBE_NODE = 80; // Dummy variable - Test this!
  public static final double EXTENDER_SM_MAX_VEL = 4000; // Dummy variable - Test this!
  public static final double EXTENDER_SM_MIN_VEL = 0; // Dummy variable - Test this!
  public static final double EXTENDER_SM_MAX_ACCEL = 8000; // Dummy variable - Test this!
  public static final double EXTENDER_SM_ALLOWED_ERR = .25; // Dummy variable - Test this!
  public static final double EXTENDER_MAX_POSTION = 100; // Max forward rotation. Current gearing is xx to one so position of xx is one full rotation of output shaft.  Only need to go about 25%
  public static final double EXTENDER_MIN_POSTION = -10; // Min forward rotation. Assuming we are starting in our minimal position of 0.
  public static final double EXTENDER_MOVEMENT_INCREMENT = .2;
  public static final int EXTENDER_MINIMUM_ARM_POSITION_TO_EXTEND = 0;

  // CTR PID Constants
  public static final int PID_CLOSED_LOOP = 0;
  public static final int PID_AUX_LOOP = 1;
  public static final int kTimeoutMs = 30;

  //  Shoulder Constants
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
  public static final double SHOULDER_HORIZONTAL_POS = 85000;
  public static final double SHOULDER_POSITION_MIN = 0;  //  Min position of the arm
  public static final double SHOULDER_POSITION_SAFE_TO_EXTEND = 40000;  //  Safe distance where can extend the rest of the arm
  public static final double SHOULDER_POSITION_MAX = 100000;  // Max position of the arm
  public static final double SHOULDER_POSITION_STOW = 0;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_PICKUP_CUBE = 5000;   // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_PICKUP_CONE = 5000;   // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_LOW_CONE_NODE = 10000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_MID_CONE_NODE = 25000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_HIGH_CONE_NODE = 70000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_LOW_CUBE_NODE = 15000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_MID_CUBE_NODE = 30000;  // Dummy variable - Test this!
  public static final double SHOULDER_POSITION_HIGH_CUBE_NODE = 75000;  // Dummy variable - Test this!
  public static final double SHOULDER_MOVEMENT_INCREMENT = 2500;

    // Wrist Constants
    public static final int WRIST_PID_SLOT = 0;
    public static final double WRIST_kF = 0.0;  // Dummy variable - Test this!
    public static final double WRIST_kP = 0.0005;  // Dummy variable - Test this!
    public static final double WRIST_kI = 0.0;  // Dummy variable - Test this!
    public static final double WRIST_kD = 0.0;  // Dummy variable - Test this!
    public static final double WRIST_kMaxOuput = 1;
    public static final double WRIST_kMinOutput = -1;
    public static final double WRIST_MAX_RPM = 5700;
    public static final double WRIST_POSITION_STOW = 0;  // Dummy variable - Test this!
    public static final double WRIST_POSITION_GROUND = 7;   // Dummy variable - Test this!
    public static final double WRIST_POSITION_LOW_CONE_NODE = 4;  // Dummy variable - Test this!
    public static final double WRIST_POSITION_MID_CONE_NODE = 4;  // Dummy variable - Test this!
    public static final double WRIST_POSITION_HIGH_CONE_NODE = 4;  // Dummy variable - Test this!
    public static final double WRIST_POSITION_LOW_CUBE_NODE = 7;  // Dummy variable - Test this!
    public static final double WRIST_POSITION_MID_CUBE_NODE = 7;  // Dummy variable - Test this!
    public static final double WRIST_POSITION_HIGH_CUBE_NODE = 55;  // Dummy variable - Test this!
    public static final double WRIST_SM_MAX_VEL = 4000;  // Dummy variable - Test this!
    public static final double WRIST_SM_MIN_VEL = 0;  // Dummy variable - Test this!
    public static final double WRIST_SM_MAX_ACCEL = 3000;  // Dummy variable - Test this!
    public static final double WRIST_SM_ALLOWED_ERR = .05;  // Dummy variable - Test this!

    public static final double WRIST_MAX_POSTION = 60; // Max forward rotation. Current gearing is 31.25 to one so position of 31.25 is one full rotation of output shaft.  Only need to go about 25%
    public static final double WRIST_MIN_POSTION = -10; // Min forward rotation. Assuming we are starting in our minimal position of 0.
    public static final double WRIST_MOVEMENT_INCREMENT = 0.5;  // Amount to move wrist postion at one time.
    public static final int WRIST_MINIMUM_ARM_POSITION_TO_EXTEND = 0;
    
}
