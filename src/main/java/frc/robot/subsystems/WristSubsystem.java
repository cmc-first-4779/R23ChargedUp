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

public class WristSubsystem extends SubsystemBase {

  // Declare our SparkMax Motor Controller
  CANSparkMax wristMotor;
  SparkMaxPIDController m_pidController;
  double setPoint;

  // Declare the variables //Not being used so commmenting out --KH
  // public double kF, kP, kI, kD, rotationsExtend, rotationsRetract;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    // Address our motor
    wristMotor = new CANSparkMax(HardwareMap.CAN_ADDRESS_WRIST, MotorType.kBrushless);
    m_pidController = wristMotor.getPIDController();

    setPoint = 0;

    // Initialize our SparkMax's to known settings
    initSparkMaxMotorController(wristMotor, "NEO550");
    // Reset our Encoder
    resetEncoder(wristMotor);
    m_pidController = wristMotor.getPIDController();
  // Config our PID Values
    configPIDFValues(wristMotor, Constants.WRIST_kP, Constants.WRIST_kI, Constants.WRIST_kD,
        Constants.WRIST_kF, Constants.WRIST_kMinOutput, Constants.WRIST_kMaxOuput);
    // Configure Smart Motion
    configureSmartMotion(wristMotor, Constants.WRIST_SM_MAX_VEL, Constants.WRIST_SM_MIN_VEL,
        Constants.WRIST_SM_MAX_ACCEL, Constants.WRIST_SM_ALLOWED_ERR, Constants.WRIST_PID_SLOT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put our Encoder Position to the SmartDashboard
    SmartDashboard.putNumber("Wrist Position", wristMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist SetPoint", setPoint);
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
    // sparkMax.burnFlash(); // Burn these settings into the flash in case of an electrical issue.
  }

  // Reset our Encoder
  public void resetEncoder(CANSparkMax sparkMax) {
    sparkMax.getEncoder().setPosition(0);
  }

  // Configure our PID Values
  public void configPIDFValues(CANSparkMax sparkMax, double kP, double kI, double kD, double kF, double kMinOutput,
      double kMaxOutput) {
        // m_pidController = wristMotor.getPIDController();
    // Configure the PID settings
    m_pidController.setFF(kF);
  System.out.println("RevError: " + m_pidController.setP(.0005));
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
    // m_pidController = sparkMax.getPIDController();
    m_pidController.setSmartMotionMaxVelocity(maxVel, slot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, slot);
    m_pidController.setSmartMotionMaxAccel(maxAccel, slot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, slot);
  }

  // Stop our Wrist Motor
  public void stopMotor() {
    wristMotor.stopMotor();
  }

  /**
   * Uses SmartMotion to set the position of the wrist to the given position
   * 
   * @param setpoint the desired position
   */
  public void setWristPosition(double setpoint) {
    // Check to make sure give position is within our allowed limits.
    if (setPointIsValid(setpoint)) {
      // send our setpoint to SmartMotion
      System.out.println("P: " + wristMotor.getPIDController().getP());
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
    if (setPoint >= Constants.WRIST_MIN_POSTION && setPoint <= Constants.WRIST_MAX_POSTION) {
      System.out.println("Setpoint is valid: " + setPoint);
      return true;
    } else {
      System.out.println("Given position " + setPoint + " is outside legal bounderies of " + Constants.WRIST_MIN_POSTION
          + " and " + Constants.WRIST_MAX_POSTION);
    }
    return false;
  }

  // Used for testing our PID settings for SmartMotion
  public void testWristPosition(double setpoint, double kF, double kP, double kI, double kD, double kMaxOutput,
      double kMinOutput, double maxVel, double minVel, double maxAccel, double allowedErr,
      int slot) {
    // Configure the PID settings
    m_pidController.setFF(kF);
    System.out.println("In TestWristPosition kP passed in: " + kP);
    System.out.println("p of pidController before setting: " + m_pidController.getP());
    m_pidController.setP(kP);
    System.out.println("p of pidController after setting: " + m_pidController.getP());
    m_pidController.setIZone(kI);
    m_pidController.setD(kD);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    // Set our crusing velocities, accell, and error.
    m_pidController.setSmartMotionMaxVelocity(maxVel, slot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, slot);
    m_pidController.setSmartMotionMaxAccel(maxAccel, slot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, slot);

    if (setPointIsValid(setpoint)) {
      setPoint = setpoint;
      // send our setpoint to SmartMotion
      m_pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  public void raiseWrist() {
    // Raising the wrist is moving it in a negative direction. Need to make sure we
    // are not lower than the minimal position
    double newSetPoint = setPoint - Constants.WRIST_MOVEMENT_INCREAMENT;
    if (setPointIsValid(newSetPoint)) {
      setPoint = newSetPoint;
      setWristPosition(setPoint);
    } else {
      System.out.println("Setpoint at it's lower limit allready: " + setPoint);
    }

  }

  public void lowerWrist() {
    // Lowering the wrist is moving it in a positive direction. Need to make sure we
    // are not higher than the minimal position
    double newSetPoint = setPoint + Constants.WRIST_MOVEMENT_INCREAMENT;
    if (setPointIsValid(newSetPoint)) {
      setPoint = newSetPoint;
      setWristPosition(setPoint);
    } else {
      System.out.println("Setpoint at it's higher limit allready: " + setPoint);
    }

  }

}
