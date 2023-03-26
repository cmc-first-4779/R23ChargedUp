// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    public static final int kOperatorControllerPort = 1;
  }

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4953; // DONE DONE Measure and set trackwidth
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_THROTTLE = 1.0; // Drivetrain throttle
  public static final double PIGEON_DEADBAND_GOING_UP = .5;
  public static final double PIGEON_DEADBAND_GOING_DOWN = -.5;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6096; // DONE Measure and set wheelbase

  public static final int DRIVETRAIN_PIGEON_ID = 5; // DONE Set Pigeon ID

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // DONE Set front left module drive motor ID
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; // DONE Set front left module steer motor ID
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21; // DOne Set front left steer encoder ID
  // public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
  // -Math.toRadians(239.22); // DONE Measure and set front
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(236.16); // DONE Measure and set front
                                                                                       // left steer offset

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // DONE Set front right drive motor ID
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; // DOne Set front right steer motor ID
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; // DONE Set front right steer encoder ID
  // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
  // -Math.toRadians(56.25); // DONE Measure and set front
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(54.49); // DONE Measure and set front
                                                                                       // right steer offset

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; // DONE Set back left drive motor ID
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 13; // DONE Set back left steer motor ID
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23; // DONE Set back left steer encoder ID
  // public static final double BACK_LEFT_MODULE_STEER_OFFSET =
  // -Math.toRadians(359.9); // DONE Measure and set back
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(2.11); // DONE Measure and set back
                                                                                    // left steer offset

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4; // DONE Set back right drive motor ID
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14; // DONE Set back right steer motor ID
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 24; // DONE Set back right steer encoder ID
  // public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
  // -Math.toRadians(136.40); // DONE Measure and set back
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(133.68); // DONE Measure and set back
                                                                                       // right steer offset

  public static final double kPXController = 0.077777;// need to change for comp bot
  public static final double kPYController = 0.077777;
  public static final double kPXYController = 0.077777;
  public static final double kPThetaController = 0.77777;

  // I'm not sure where these values come from, but this is what 0 to auto code is
  // using.
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond,
          kMaxAngularAccelerationRadiansPerSecondSquared);

  // Deadzone for auto balancing
  public static final double DRIVETRAIN_AUTOBALANCE_DEADZONE_Y = 7.0;
  public static final double DRIVETRAIN_AUTOBALANCE_DEADZONE_X = 7.0;
  public static final double DRIVETRAIN_AUTOBALANCE_THROTTLE = 1.85;  // Old:  1.75;

  // Intake Constants
  public static final double INTAKE_CONE_SPEED = -0.80;
  public static final double INTAKE_CUBE_SPEED = 0.60;
  public static final double EJECT_CONE_SPEED = 0.70;
  public static final double EJECT_CUBE_SPEED = -0.50;
  public static final double INTAKE_CONE_AUTON_TIMER = 0.25; // Timer for Intake Cone in Auton
  public static final double INTAKE_CUBE_AUTON_TIMER = 3.00; // Timer for Intake Cube in Auton
  public static final double EJECT_CONE_AUTON_TIMER = 0.50;
  public static final double EJECT_CUBE_AUTON_TIMER = 0.5;
  public static final double INTAKE_CURRENT_WO_LOAD = 4.0;  //dummy value
  public static final double INTAKE_CURRENT_WITH_CUBE = 17.0; 
  public static final double INTAKE_CURRENT_WITH_CONE = 20.0;
  public static final double INTAKE_DEBOUNCE_SECONDS = 0.5; // period of time we look at the current
  public static final int INTAKE_FILTER_TAPS = 50;  //  Number of samples of our current needed

  // Extender Arm Constants
  public static final int EXTENDER_PID_SLOT = 0;
  public static final double EXTENDER_kF = 0.0;
  public static final double EXTENDER_kP = 0.075;
  public static final double EXTENDER_kI = 0.0;
  public static final double EXTENDER_kD = 0.0;
  public static final double EXTENDER_kMaxOuput = 1;
  public static final double EXTENDER_kMinOutput = -1;
  public static final double EXTENDER_NOMINAL_OUTPUT_FORWARD = 0.0;
  public static final double EXTENDER_NOMINAL_OUTPUT_REVERSE = 0.0;
  public static final double EXTENDER_PEAK_OUTPUT_FORWARD = 1.0;
  public static final double EXTENDER_PEAK_OUTPUT_REVERSE = -1.0;
  public static final double EXTENDER_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS = .5; // Dummy variable - Test this!
  public static final double EXTENDER_NEUTRAL_DEADBAND_PERCENT = 0.04; // 0.04 is the default
  public static final double EXTENDER_MAX_RPM = 5700;
  public static final double EXTENDER_MM_MAX_VEL = 20000; // Dummy variable - Test this!
  public static final double EXTENDER_MM_MAX_ACCEL = 20000; // Dummy variable - Test this!
  public static final double EXTENDER_MM_ALLOWED_ERR = 250; // Dummy variable - Test this!
  public static final double EXTENDER_MAX_POSTION = 179000; // Max forward rotation. Current gearing is xx to one so
                                                            // position of xx is one full rotation of output shaft. Only
                                                            // need to go about 25%
  public static final double EXTENDER_MIN_POSTION = -0; // Min forward rotation. Assuming we are starting in our
                                                        // minimal position of 0.
  public static final double EXTENDER_MOVEMENT_INCREMENT = 2000;
  public static final int EXTENDER_MINIMUM_ARM_POSITION_TO_EXTEND = 0;

  // CTR PID Constants
  public static final int PID_CLOSED_LOOP = 0;
  public static final int PID_AUX_LOOP = 1;
  public static final int kTimeoutMs = 30;

  // Shoulder Constants
  public static final double SHOULDER_MM_VELOCITY = 16384; // Dummy variable - Test this!
  public static final double SHOULDER_MM_ACCELERATION = 16384; // Dummy variable - Test this!
  public static final double SHOULDER_MM_VELOCITY_DOWN = 9830;
  public static final double SHOULDER_MM_ACCELERATION_DOWN = 9830;

  public static final double SHOULDER_NOMINAL_OUTPUT_FORWARD = 0.0;
  public static final double SHOULDER_NOMINAL_OUTPUT_REVERSE = 0.0;
  public static final double SHOULDER_PEAK_OUTPUT_FORWARD = 1.0;
  public static final double SHOULDER_PEAK_OUTPUT_REVERSE = -1.0;
  public static final double SHOULDER_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS = .5; // Dummy variable - Test this!
  public static final double SHOULDER_NEUTRAL_DEADBAND_PERCENT = 0.04; // 0.04 is the default
  public static final double SHOULDER_ALLOWED_ERROR = 250;
  public static final double SHOULDER_DEFAULT_kP = 0.093658; // was 0.05
  public static final double SHOULDER_DEFAULT_kI = 0.00;
  public static final double SHOULDER_DEFAULT_kD = 0.00;
  public static final double SHOULDER_DEFAULT_kF = 0.056854; // was 0.00;
  public static final double SHOULDER_DEFAULT_kP_slot1 = 0.093658; // was 0.03
  public static final double SHOULDER_DEFAULT_kI_slot1 = 0.00;
  public static final double SHOULDER_DEFAULT_kD_slot1 = 0.00;
  public static final double SHOULDER_DEFAULT_kF_slot1 = 0.056854; // was 0.00;
  public static final double SHOULDER_MAX_GRAVITY_kF = 0.07;
  public static final double SHOULDER_HORIZONTAL_POS = 85000;
  public static final double SHOULDER_POSITION_MIN = 0; // Min position of the arm
  public static final double SHOULDER_POSITION_MAX = 92500; // Max position of the arm
  public static final double SHOULDER_MOVEMENT_INCREMENT = 2500;
  public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = 0.6069; // Absolute position of encoder when arm is at 0
                                                                       // between 0.0 and 1;

  // Wrist Constants
  public static final int WRIST_PID_SLOT = 0;
  public static final double WRIST_kF = 0.0; // Dummy variable - Test this!
  public static final double WRIST_kP = 0.0005; // Dummy variable - Test this!
  public static final double WRIST_kI = 0.0; // Dummy variable - Test this!
  public static final double WRIST_kD = 0.0; // Dummy variable - Test this!
  public static final double WRIST_kMaxOuput = 1;
  public static final double WRIST_kMinOutput = -1;
  public static final double WRIST_MAX_RPM = 5700;
  public static final double WRIST_SM_MAX_VEL = 7500; // Dummy variable - Test this!
  public static final double WRIST_SM_MIN_VEL = 0; // Dummy variable - Test this!
  public static final double WRIST_SM_MAX_ACCEL = 6000; // Dummy variable - Test this!
  public static final double WRIST_SM_ALLOWED_ERR = 0.5; // Dummy variable - Test this!
  public static final double WRIST_MAX_POSTION = 84.8; // Max forward rotation. Current gearing is 31.25 to one so
                                                       // position of 31.25 is one full rotation of output shaft. Only
                                                       // need to go about 25%
  public static final double WRIST_MIN_POSTION = -10; // Min forward rotation. Assuming we are starting in our minimal
                                                      // position of 0.
  public static final double WRIST_MOVEMENT_INCREMENT = 0.5; // Amount to move wrist postion at one time.
  public static final int WRIST_MINIMUM_ARM_POSITION_TO_EXTEND = 0;
  public static final double WRIST_ABSOLUTE_ENCODER_OFFSET = 0.6069;

  // LIMELIGHT Constants
  // Tolerance of Degrees we can be off on the x-axis
  public static final double LIMELIGHT_X_DRIVE_TOLERANCE = 0.10;
  // Tolerance of Degrees we can be off on the y-axis
  public static final double LIMELIGHT_Y_DRIVE_TOLERANCE = 0.30;
  //  Minimum throttle it takes to move the robot.
  public static final double LIMELIGHT_MIN_MOVE = 0.20;

}
