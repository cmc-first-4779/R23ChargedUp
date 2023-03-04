// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.StaticConstants.HardwareMap;
import frc.StaticConstants.MaxMotorAmpsConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

//  This Subsystem is for our Extender Arm and will be powered by one Spark Neo550;

public class ExtenderSubsystem extends SubsystemBase {

  // Declare our SparkMax Motor Controller
  CANSparkMax extenderMotor;
  SparkMaxPIDController m_pidController;

  // Declare the variables
  public double kF, kP, kI, kD, rotationsExtend, rotationsRetract;
  double setPoint;

  // Reference to robot container to access other subsystems
  RobotContainer robotContainer;

  /** Creates a new ExtenderArmSubsystem. */
  public ExtenderSubsystem(RobotContainer robotContainer) {
    // Address our motor
    extenderMotor = new CANSparkMax(HardwareMap.CAN_ADDRESS_EXTENDER_ARM, MotorType.kBrushless);
    m_pidController = extenderMotor.getPIDController();
    this.robotContainer = robotContainer;
    setPoint = 0;

    kF = Constants.EXTENDER_kF;
    kP = Constants.EXTENDER_kP;
    kI = Constants.EXTENDER_kI;
    kD = Constants.EXTENDER_kD;
    // Initialize our SparkMax's to known settings
    initSparkMaxMotorController(extenderMotor, "NEO");
    // Reset our Encoder
    resetEncoder(extenderMotor);
    // Config our PID Values
    configPIDFValues(extenderMotor, kP, kI, kD, kF, Constants.EXTENDER_kMinOutput, Constants.EXTENDER_kMaxOuput);
    // Configure Smart Motion
    configureSmartMotion(extenderMotor, Constants.EXTENDER_SM_MAX_VEL, Constants.EXTENDER_SM_MIN_VEL,
        Constants.EXTENDER_SM_MAX_ACCEL, Constants.EXTENDER_SM_ALLOWED_ERR, Constants.EXTENDER_PID_SLOT);
    //

    // Add PID Fields to SmartDashboard
    SmartDashboard.putNumber("Position", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put our Encoder Position to the SmartDashboard
    SmartDashboard.putNumber("Extender Position", extenderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Extender Setpoint", setPoint);
    SmartDashboard.putNumber("Extender Velocity", extenderMotor.getEncoder().getVelocity());
  }

  // Initialize a SparkMax Motor controller and set our default settings.
  private static void initSparkMaxMotorController(CANSparkMax sparkMax, String Motortype) {
    System.out.println("Initializing SparkMax: " + sparkMax);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setIdleMode(IdleMode.kBrake); // kCoast is Coast... kBrake is Brake
    if (Motortype == "NEO550") {
      sparkMax.setSmartCurrentLimit(MaxMotorAmpsConstants.MAX_AMPS_STATOR_LIMIT_NEO550); // Set the Amps limit
    } else {
      sparkMax.setSmartCurrentLimit(MaxMotorAmpsConstants.MAX_AMPS_STATOR_LIMIT_NEO); // Set the Amps limit
    }
    // sparkMax.burnFlash(); // Burn these settings into the flash in case of an
    // electrical issue.
  }

  // Reset our Encoder
  public void resetEncoder(CANSparkMax sparkMax) {
    sparkMax.getEncoder().setPosition(0);
  }

  // Configure our PID Values
  public void configPIDFValues(CANSparkMax sparkMax, double kP, double kI, double kD, double kF, double kMinOutput,
      double kMaxOutput) {
    // Configure the PID settings
    m_pidController.setFF(kF);
    m_pidController.setP(kP);
    m_pidController.setIZone(kI);
    m_pidController.setD(kD);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  // Configure our Smart Motion settings
  /**
   * Smart Motion coefficients are set on a SparkMaxPIDController object
   * 
   * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
   * the pid controller in Smart Motion mode
   * - setSmartMotionMinOutputVelocity() will put a lower bound in
   * RPM of the pid controller in Smart Motion mode
   * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
   * of the pid controller in Smart Motion mode
   * - setSmartMotionAllowedClosedLoopError() will set the max allowed
   * error for the pid controller in Smart Motion mode
   */
  public void configureSmartMotion(CANSparkMax sparkMax, double maxVel, double minVel, double maxAccel,
      double allowedErr, int slot) {
    m_pidController.setSmartMotionMaxVelocity(maxVel, slot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, slot);
    m_pidController.setSmartMotionMaxAccel(maxAccel, slot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, slot);
  }

  // Stop our Extender Motor
  public void stopMotor() {
    extenderMotor.stopMotor();
  }

  public void setExtenderPosition(double setpoint) {
    if (setPointIsValid(setpoint)) {
      // Declare our PID Controller
      System.out.println("P is:  " + m_pidController.getP());
      System.out.println("Setpoint is:  " + setpoint);
      // send our setpoint to SmartMotion
      m_pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  /**
   * Checks to see if the provided setPoint within the legal bounds
   * 
   * @param setPoint
   * @return true if it falls on or between min and max allowed values.
   */
  private boolean setPointIsValid(double setPoint) {
    if ((setPoint >= Constants.EXTENDER_MIN_POSTION) && (extenderMotor.get() >= 0)) {
      System.out.println("Setpoint is valid: " + setPoint);
      return true;
    } else if ((setPoint <= Constants.EXTENDER_MAX_POSTION) && extenderMotor.get() <= 0) {
      System.out.println("Setpoint is valid: " + setPoint);
      return true;
    } else {
      System.out
          .println("Given position " + setPoint + " is outside legal bounderies of " + Constants.EXTENDER_MIN_POSTION);
      return false;
    }
  }

  // Used for testing our PID settings for SmartMotion
  public void testExtenderPosition(double setpoint, double kF, double kP, double kI, double kD, double kMaxOutput,
      double kMinOutput, double maxVel, double minVel, double maxAccel, double allowedErr,
      int slot) {
    // Declare our PID Controller
    // Configure the PID settings
    m_pidController.setFF(kF);
    m_pidController.setP(kP);
    m_pidController.setIZone(kI);
    m_pidController.setD(kD);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    // Set our crusing velocities, accell, and error.
    m_pidController.setSmartMotionMaxVelocity(maxVel, slot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, slot);
    m_pidController.setSmartMotionMaxAccel(maxAccel, slot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, slot);
    // send our setpoint to SmartMotion
    if (setPointIsValid(setpoint)) {
      m_pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    } else {
      System.out.println("Setpoint: " + setpoint + " is not valid");
    }
  }

  /**
   * Retracts the wrist by the WRIST_MOVEMENT_INCREMENT
   */
  public void retractExtender() {
    // Retracting the arm is moving it in a negative direction. Need to make sure
    // we are not lower than the minimal position
    double newSetPoint = setPoint - Constants.EXTENDER_MOVEMENT_INCREMENT;
    // if (setPointIsValid(newSetPoint)) {
    // Since we are retracting, we only have to make sure the new setpoint is
    // greater than the minimum.
    if (newSetPoint > Constants.EXTENDER_MIN_POSTION) {
      setPoint = newSetPoint;
      setExtenderPosition(setPoint);
    } else {
      System.out.println("Setpoint at it's lower limit allready: " + setPoint);
    }
  }

  /**
   * Extends the wrist by the WRIST_MOVEMENT_INCREMENT
   */
  public void extendExtender() {
    // Extending the armis moving it in a positive direction. Need to make sure we
    // are not greater than max allowed distance.
    double newSetPoint = setPoint + Constants.EXTENDER_MOVEMENT_INCREMENT;
    if (!safeToExtendExtender()) {
      System.out.println("Arm is not at safe position to extend wrist");
      return;
    }
    // Check to see that we are not past max position already
    // if (setPointIsValid(newSetPoint)) {
    if (newSetPoint < Constants.EXTENDER_MAX_POSTION) {
      setPoint = newSetPoint;
      setExtenderPosition(setPoint);
    } else {
      System.out.println("Setpoint at it's higher limit allready: " + setPoint);
    }

  }

  /**
   * Checks to see if the Arm is in a safe position for the wrist to extend
   * 
   * @return true if arm position is greater than minimum distance set to extend
   */
  private boolean safeToExtendExtender() {
    // Need to check with arm to make sure it's in a good space.
    double shoulderPosition = robotContainer.getShoulderPosition();
    if (shoulderPosition > Constants.SHOULDER_POSITION_SAFE_TO_EXTEND) {
      System.out.println("Shoulder is at safe position to extend Extender:  " +shoulderPosition);
      return true;
    }
    System.out.println("Shoulder is NOT at safe position to extend Extender:  "+shoulderPosition);
    return false;
  }

  /**
   * Extends and retracts the arm based off of given speed.
   * 
   * @param speed
   */
  public void moveExtender(double speed) {
    // Joystick method to move the extender manually
    // Make sure joystick was actually moved enough to register
    if (Math.abs(speed) > .1) {
      // Chekc to see if we are trying to extend
      if (speed > 0) {
        if (safeToExtendExtender() && extenderMotor.getEncoder().getPosition() < Constants.EXTENDER_MAX_POSTION) {
          extenderMotor.set(speed);
        } else {
          stopMotor();
        }
      } else { // Attempting to retract
        if (extenderMotor.getEncoder().getPosition() > Constants.EXTENDER_MIN_POSTION) {
          extenderMotor.set(speed);
        } else {
          stopMotor();
        }
      }
    } else {
      stopMotor();
    }
  }

  public double getExtenderPosition() {
    return extenderMotor.getEncoder().getPosition();
}

}
