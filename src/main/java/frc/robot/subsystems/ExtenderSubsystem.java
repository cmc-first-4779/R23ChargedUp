// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PositionSetpoints;
import frc.robot.RobotContainer;
import frc.robot.StaticConstants.HardwareMap;
import frc.robot.StaticConstants.MaxMotorAmpsConstants;

//  This Subsystem is for our Extender Arm and will be powered by one Spark Neo550;

public class ExtenderSubsystem extends SubsystemBase {

  // Declare our SparkMax Motor Controller
  WPI_TalonFX extenderMotor;
 

  // Declare the variables
  public double kF, kP, kI, kD, rotationsExtend, rotationsRetract;
  double setPoint;

  // Reference to robot container to access other subsystems
  RobotContainer robotContainer;

  /** Creates a new ExtenderArmSubsystem. */
  public ExtenderSubsystem(RobotContainer robotContainer) {
    // Address our motor
    // extenderMotor = new CANSparkMax(HardwareMap.CAN_ADDRESS_EXTENDER_ARM,
    // MotorType.kBrushless);
    extenderMotor = new WPI_TalonFX(HardwareMap.CAN_ADDRESS_EXTENDER_ARM);
    // m_pidController = extenderMotor.getPIDController();
    this.robotContainer = robotContainer;
    setPoint = 0;

    kF = Constants.EXTENDER_kF;
    kP = Constants.EXTENDER_kP;
    kI = Constants.EXTENDER_kI;
    kD = Constants.EXTENDER_kD;
    // Initialize our SparkMax's to known settings
    initMotorController(extenderMotor);
    // Reset our Encoder
    resetEncoder(extenderMotor);
    configSimpleMM(extenderMotor);



    //

    // Add PID Fields to SmartDashboard
   // SmartDashboard.putNumber("Position", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put our Encoder Position to the SmartDashboard
    //SmartDashboard.putNumber("Extender Position", extenderMotor.getSelectedSensorPosition());
   // SmartDashboard.putNumber("Extender Setpoint", setPoint);
    //SmartDashboard.putNumber("Extender Velocity", extenderMotor.getSelectedSensorVelocity());
  }

  // Initialize a TalonFX Motor controller and set our default settings.
  private static void initMotorController(WPI_TalonFX talon) {
    System.out.println("Initializing Falcon: " + talon);
    // Set factory defaults
    talon.configFactoryDefault();
    // Set Neutral Mode
    talon.setNeutralMode(NeutralMode.Brake); // Neutral Mode is Brake
    // Netural the controller output and disable it until we call it later
    talon.neutralOutput();
    // Config the neutral deadband
    talon.configNeutralDeadband(Constants.EXTENDER_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS, Constants.kTimeoutMs);
    talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.kTimeoutMs);
    talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.kTimeoutMs);
    /* Set the peak and nominal outputs */
    talon.configNominalOutputForward(Constants.EXTENDER_NOMINAL_OUTPUT_FORWARD, Constants.kTimeoutMs);
    talon.configNominalOutputReverse(Constants.EXTENDER_NOMINAL_OUTPUT_REVERSE, Constants.kTimeoutMs);
    talon.configPeakOutputForward(Constants.EXTENDER_PEAK_OUTPUT_FORWARD, Constants.kTimeoutMs);
    talon.configPeakOutputReverse(Constants.EXTENDER_PEAK_OUTPUT_REVERSE, Constants.kTimeoutMs);
    // Set how many seconds that the motor can ramp from neutral to full
    talon.configClosedloopRamp(Constants.EXTENDER_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS, Constants.kTimeoutMs);
    /**
     * Configure the current limits that will be used
     * Stator Current is the current that passes through the motor stators.
     * Use stator current limits to limit rotor acceleration/heat production
     * Supply Current is the current that passes into the controller from the supply
     * Use supply current limits to prevent breakers from tripping
     *
     * enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
     */
    // talon.configStatorCurrentLimit(
    new StatorCurrentLimitConfiguration(true, MaxMotorAmpsConstants.MAX_AMPS_STATOR_LIMIT_FALCON500,
        MaxMotorAmpsConstants.MAX_AMPS_STATOR_TRIGGER_FALCON500,
        MaxMotorAmpsConstants.MAX_SECS_STATOR_THRESHOLDTIME_FALCON500);
  }

  // Resets our Encoder to ZERO
  public void resetEncoder(WPI_TalonFX talon) {
    talon.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    // System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
  }

  // Configure our PID Values
  public void configPIDFValues(WPI_TalonFX talon, double p, double i, double d, double f, int slot) {
    // Configure the PID settings for Slot0
    talon.config_kF(slot, f);
    talon.config_kP(slot, p);
    talon.config_kI(slot, i);
    talon.config_kD(slot, d);
  }

  private void configMotionCruiseAndAcceleration(WPI_TalonFX talon, double velocity, double acceleration) {
    // Motion Magic needs a CruiseVelocity and Acceleration that needs to be set.
    // The value is is sensor units/100ms. So a CIM Encoder has 80 units per
    // rotation and an AM Mag Encoder has 4096 units per rotation so this value will
    // be way differnent. A good way to figure this out is to use Phoenix Tuner to
    // run the motor at 100% and do a selftest snapshot to get the velocity which
    // will be in units/100ms. This is the max velocity the motor can do in it's
    // current configuration.
    talon.configMotionCruiseVelocity(velocity, Constants.kTimeoutMs);
    talon.configMotionAcceleration(acceleration, Constants.kTimeoutMs);
  }

  private void configAllowableError(WPI_TalonFX talon, int slot, double allowedError) {
    // Configure the closeed loop error, which is how close the sensor has to be to
    // target to be successful.
    talon.configAllowableClosedloopError(slot, allowedError, Constants.kTimeoutMs);
  }

    // Configure our Motion Magic PID Loop
    private void configSimpleMM(WPI_TalonFX talon) {
      // Tell each talon to use Quad Encoder as their PID0
      talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_CLOSED_LOOP,
          Constants.kTimeoutMs);
      // Talons have 4 slots of PID variables and 2 PID indexes. Set the PID0 to use
      // Slot0
      talon.selectProfileSlot(0, 0);
      // Set up PID Values for the Winch
      configPIDFValues(talon, 0.075, kI, kD, kF, 0); // STILL NEED TO GET THESE VALUES
      configMotionCruiseAndAcceleration(talon, Constants.EXTENDER_MM_MAX_VEL, Constants.EXTENDER_MM_MAX_ACCEL);
      configAllowableError(talon, 0, Constants.EXTENDER_MM_ALLOWED_ERR);
      talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
    }

  // Stop our Extender Motor
  public void stopMotor() {
    extenderMotor.stopMotor();
  }
  
  //  Sets the Extender Position
  public void setExtenderPosition(double setpoint) {
    
    if (setPointIsValid(setpoint)) {
      // Declare our PID Controller
     // System.out.println("P is:  " + extenderMotor.getP
      System.out.println("Setpoint is:  " + setpoint);
      // send our setpoint to SmartMotion
      extenderMotor.setSafetyEnabled(false);
      extenderMotor.set(TalonFXControlMode.MotionMagic, setpoint);
    }
  }

  /**
   * Checks to see if the provided setPoint within the legal bounds
   * 
   * @param setPoint
   * @return true if it falls on or between min and max allowed values.
   */
  private boolean setPointIsValid(double setPoint) {
    System.out.println("Extender Current Pos:  " +extenderMotor.getSelectedSensorPosition());
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
      System.out.println("Setpoint at it's higher limit already: " + setPoint);
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
    if (shoulderPosition > PositionSetpoints.SHOULDER_POSITION_SAFE_TO_EXTEND) {
      System.out.println("Shoulder is at safe position to extend Extender:  " + shoulderPosition);
      return true;
    }
    System.out.println("Shoulder is NOT at safe position to extend Extender:  " + shoulderPosition);
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
      // Check to see if we are trying to extend
      if (speed > 0) {
        if (safeToExtendExtender() && extenderMotor.getSelectedSensorPosition() < Constants.EXTENDER_MAX_POSTION) {
          extenderMotor.set(speed);
        } else {
          stopMotor();
        }
      } else { // Attempting to retract
        if (extenderMotor.getSelectedSensorPosition() > Constants.EXTENDER_MIN_POSTION) {
          extenderMotor.set(speed);
        } else {
          stopMotor();
        }
      }
    } else {
      stopMotor();
    }
  }

  //  Returns the Extender Position when called
  public double getExtenderPosition() {
    return extenderMotor.getSelectedSensorPosition();
  }

}
