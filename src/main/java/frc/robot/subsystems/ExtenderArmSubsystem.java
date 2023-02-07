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

//  This Subsystem is for our Extender Arm and will be powered by one Spark Neo550;

public class ExtenderArmSubsystem extends SubsystemBase {

  // Declare our SparkMax Motor Controller
  CANSparkMax extenderMotor;

  // Declare the variables
  public double kF, kP, kI, kD, rotationsExtend, rotationsRetract;

  /** Creates a new ExtenderArmSubsystem. */
  public ExtenderArmSubsystem() {
    // Address our motor
    extenderMotor = new CANSparkMax(HardwareMap.CAN_ADDRESS_EXTENDER_ARM, MotorType.kBrushless);

    // Initialize our SparkMax's to known settings
    initSparkMaxMotorController(extenderMotor, "NEO550");
    // Reset our Encoder
    resetEncoder(extenderMotor);
    // Config our PID Values
    configPIDFValues(extenderMotor, Constants.EXTENDER_kP, Constants.EXTENDER_kI, Constants.EXTENDER_kD,
        Constants.EXTENDER_kF, Constants.EXTENDER_kMinOutput, Constants.EXTENDER_kMaxOuput);
    // Configure Smart Motion
    configureSmartMotion(extenderMotor, Constants.EXTENDER_SM_MAX_VEL, Constants.EXTENDER_SM_MIN_VEL,
        Constants.EXTENDER_SM_MAX_ACCEL, Constants.EXTENDER_SM_ALLOWED_ERR, Constants.EXTENDER_PID_SLOT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put our Encoder Position to the SmartDashboard
    SmartDashboard.putNumber("Extender Position", extenderMotor.getEncoder().getPosition());
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
    sparkMax.burnFlash(); // Burn these settings into the flash in case of an electrical issue.
  }

  // Reset our Encoder
  public void resetEncoder(CANSparkMax sparkMax) {
    sparkMax.getEncoder().setPosition(0);
  }

  // Configure our PID Values
  public void configPIDFValues(CANSparkMax sparkMax, double kP, double kI, double kD, double kF, double kMinOutput,
      double kMaxOutput) {
    // Declare our PID Controller
    SparkMaxPIDController m_pidController = sparkMax.getPIDController();
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
    // Declare our PID Controller
    SparkMaxPIDController m_pidController = sparkMax.getPIDController();
    m_pidController.setSmartMotionMaxVelocity(maxVel, slot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, slot);
    m_pidController.setSmartMotionMaxAccel(maxAccel, slot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, slot);
  }

  // Stop our Extender Motor
  public void stopExtender() {
    extenderMotor.stopMotor();
  }

  public void setExtenderPosition(double setpoint) {
    // Declare our PID Controller
    SparkMaxPIDController m_pidController = extenderMotor.getPIDController();
    // send our setpoint to SmartMotion
    m_pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
  }

}
