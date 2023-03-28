// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.StaticConstants.HardwareMap;
import frc.robot.StaticConstants.MaxMotorAmpsConstants;

public class WristSubsystem extends SubsystemBase {

  // Declare our SparkMax Motor Controller
  CANSparkMax wristMotor;
  SparkMaxPIDController m_pidController;
  double setPoint;

  // Reference to robot container to access other subsystems
  RobotContainer robotContainer;

    // Absolute Position
    double initialAbsolutePosition, referencePosition, relativeOffset;
    DutyCycleEncoder absoluteEncoder;



  /**
   * Constructor for the wrist subsystem. Sets up the motor and pid controller
   * 
   * @param robotContainer reference to the robot used to get information about
   *                       other subystems
   */
  public WristSubsystem(RobotContainer robotContainer) {
    // Address our motor
    wristMotor = new CANSparkMax(HardwareMap.CAN_ADDRESS_WRIST, MotorType.kBrushless);

    m_pidController = wristMotor.getPIDController();
    this.robotContainer = robotContainer;

    setPoint = 0;

    // Initialize our SparkMax's to known settings
    initSparkMaxMotorController(wristMotor, "NEO550");
    // Reset our Encoder
    resetEncoder(wristMotor);
    m_pidController = wristMotor.getPIDController();
    wristMotor.setInverted(true);

          // Address our Absolute encoder
  absoluteEncoder=new DutyCycleEncoder(HardwareMap.DIO_PORT_WRIST_ABSOLUTE_ENCODER);

  // Init our settings for our encoder
  initAbsoluteEncoder(absoluteEncoder);

    // Get the absolute position of the shoulder at RobotInit
    initialAbsolutePosition = absoluteEncoder.getAbsolutePosition();
    referencePosition = initialAbsolutePosition; // Safe this also in another variable for later

    // Config our PID Values
    configPIDFValues(wristMotor, Constants.WRIST_kP, Constants.WRIST_kI, Constants.WRIST_kD,
        Constants.WRIST_kF, Constants.WRIST_kMinOutput, Constants.WRIST_kMaxOuput);
    // Configure Smart Motion
    configureSmartMotion(wristMotor, Constants.WRIST_SM_MAX_VEL, Constants.WRIST_SM_MIN_VEL,
        Constants.WRIST_SM_MAX_ACCEL, Constants.WRIST_SM_ALLOWED_ERR, Constants.WRIST_PID_SLOT);

    // Add PID Fields to SmartDashboard
    // SmartDashboard.putNumber("Position", 0);
    // SmartDashboard.putNumber("kF", Constants.WRIST_kF);
    // SmartDashboard.putNumber("kP", Constants.WRIST_kP);
    // SmartDashboard.putNumber("kI", Constants.WRIST_kI);
    // SmartDashboard.putNumber("kD", Constants.WRIST_kD);
    // SmartDashboard.putNumber("Max Vel", Constants.WRIST_SM_MAX_VEL);
    // SmartDashboard.putNumber("Min Vel", Constants.WRIST_SM_MIN_VEL);
    // SmartDashboard.putNumber("Max Accel", Constants.WRIST_SM_MAX_ACCEL);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put our Encoder Position to the SmartDashboard
    SmartDashboard.putNumber("Wrist Position", wristMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist Absolute Position", absoluteEncoder.getAbsolutePosition());
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
   * Set ths motor to move at the provided speed
   * 
   * @param speed
   */
  public void moveWrist(double speed) {
    if (Math.abs(speed) > .1) {
      wristMotor.set(speed);
    } else {
      wristMotor.set(0);
    }
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
    if ((setPoint >= Constants.WRIST_MIN_POSTION) && (wristMotor.get() >= 0)) {
      System.out.println("Setpoint is valid: " + setPoint);
      return true;
    } else if ((setPoint <= Constants.WRIST_MAX_POSTION) && wristMotor.get() <= 0) {
      System.out.println("Setpoint is valid: " + setPoint);
      return true;
    } else {
      System.out
          .println("Given position " + setPoint + " is outside legal bounderies of " + Constants.WRIST_MIN_POSTION);
      return false;
    }
  }

  /**
   * Used for testing our PID settings for SmartMotion by grabbing values from the
   * dashboard
   * 
   * @param setpoint   The poisiton to set the wrist position to
   * @param kF         feed forward constant for pid controlle
   * @param kP         p constant for pid controller
   * @param kI         i constant for pid controller
   * @param kD         d cosntant for pid controller
   * @param kMaxOutput maximum output value for pid controller
   * @param kMinOutput minimum output value for pid controller
   * @param maxVel     maxium velocity for wrist to move during smart motion
   * @param minVel     minium velocity for wrist to move during smart motion
   * @param maxAccel   maximum acceleration we want to the wrist to accelerate
   *                   during smart motion
   * @param allowedErr positional distance difference the wrist can be at and
   *                   still be considered on target
   * @param slot       the pid slot to use for smart motion
   */
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

  /**
   * Retracts the wrist by the WRIST_MOVEMENT_INCREMENT
   */
  public void retractWrist() {
    // Retracting the wrist is moving it in a negative direction. Need to make sure
    // we
    // are not lower than the minimal position
    double newSetPoint = setPoint - Constants.WRIST_MOVEMENT_INCREMENT;
    if (setPointIsValid(newSetPoint)) {
      setPoint = newSetPoint;
      setWristPosition(setPoint);
    } else {
      System.out.println("Setpoint at it's lower limit allready: " + setPoint);
    }
  }

  /**
   * Extends the wrist by the WRIST_MOVEMENT_INCREMENT
   */
  public void extendWrist() {
    // Lowering the wrist is moving it in a positive direction. Need to make sure we
    // are not higher than the minimal position
    double newSetPoint = setPoint + Constants.WRIST_MOVEMENT_INCREMENT;
    if (!safeToExtendWrist()) {
      System.out.println("Arm is not at safe position to extend wrist");
      return;
    }

    if (setPointIsValid(newSetPoint)) {
      setPoint = newSetPoint;
      setWristPosition(setPoint);
    } else {
      System.out.println("Setpoint at it's higher limit allready: " + setPoint);
    }

  }

  /**
   * Checks to see if the Arm is in a safe position for the wrist to extend
   * 
   * @return true if arm position is greater than minimum distance set to extend
   */
  private boolean safeToExtendWrist() {
    // Need to check with arm to make sure it's in a good space.
    double shoulderPosition = robotContainer.getShoulderPosition();
    if (shoulderPosition > Constants.WRIST_MINIMUM_ARM_POSITION_TO_EXTEND) {
      System.out.println("Shoulder is at safe position to extend wrist");
      return true;
    }
    return false;
  }

  // Return the position of the wrist
  public double getWristPosition() {
    return wristMotor.getEncoder().getPosition();
  }

  // Return the encoder
  public RelativeEncoder getWristEncoder() {
    return wristMotor.getEncoder();
  }

    // Setup the Through Bore Encoder from REV's Specsheet:
  // https://docs.revrobotics.com/through-bore-encoder/specifications
  private void initAbsoluteEncoder(DutyCycleEncoder encoder) {
    encoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0); // PERIOD = 1025 for the Encoder
    encoder.setDistancePerRotation(125); // 125 Gear reduction ration
    encoder.setPositionOffset(Constants.WRIST_ABSOLUTE_ENCODER_OFFSET);
  }

    /**
   * Gets the current position of the shoulder motors based on the readings of the
   * Absolute encoder
   * taking into account the original 0 location by having the Absolute encoder
   * positionOffset set to
   * the absolute position of the encoder when arm was at 0.
   * 
   * @return
   */
  public double getCurrentAbosoluteDistance() {
    // Absolute ecoder is mounted inverted so negating the return value
    return -absoluteEncoder.getDistance();
  }

    // Resets our Encoder to ZERO
    public void resetEncoders(CANSparkMax sparkMax) {
      // Check to see if absolute encoder is present and use it's position if so.
      if (absoluteEncoder.isConnected()) {
        // Get the current distance of the shoulder calculated based off of absolute encoder.
        double currentMotorPositon = getCurrentAbosoluteDistance() * -1; // Negating to change phase
  
        // Check to make sure it's a reasonable number in case the encoder crossed over
        // the 0 line i.e. is reading 0.99
        if (currentMotorPositon < -100) {
          System.out.println("Encoder was giving an excessively high negative distance so normalizing");
          currentMotorPositon = currentMotorPositon + (2048 * 192);
        }
        sparkMax.getEncoder().setPosition(currentMotorPositon);
      } else {
        sparkMax.getEncoder().setPosition(0);
      }
    }

}
