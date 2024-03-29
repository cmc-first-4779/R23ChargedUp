// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PositionSetpoints;
import frc.robot.StaticConstants.HardwareMap;
import frc.robot.StaticConstants.MaxMotorAmpsConstants;

/*  This Subsystem is for our Winch that raises and lowers the arm to certain 
 *  set positions.
 * 
 *   We are assuming two Falcon 500 Motors with TalonFX Controllers
 *   Using Motion Magic to drive the arm.
 */

public class ShoulderSubsystem extends SubsystemBase {

  /** Creates a new ExtenderSubsystem. */
  // Declare our TalonFX Motor Controllers
  WPI_TalonFX shoulderMotorSlave;
  WPI_TalonFX shoulderMotorMaster;
  // Declare our Encoders
  RelativeEncoder shoulderEncoderSlave;
  RelativeEncoder shoulderEncoderMaster;
  // Absolute Duty-Cycle Encoder
  DutyCycleEncoder absoluteEncoder;

  // Encoder Position
  double shoulderMasterPosition, shoulderSlavePosition;

  // Absolute Position
  double initialAbsolutePosition, referencePosition, relativeOffset;

  // Our Setpoint for the Shoulder Position
  double setPoint;

  /** Creates a new WinchPulleySubsystem. */
  public ShoulderSubsystem() {
    // Address our controllers
    shoulderMotorSlave = new WPI_TalonFX(HardwareMap.CAN_ADDRESS_SHOULDER_MOTOR_LEFT);
    shoulderMotorMaster = new WPI_TalonFX(HardwareMap.CAN_ADDRESS_SHOULDER_MOTOR_RIGHT);
    // Address our Absolute encoder
    absoluteEncoder = new DutyCycleEncoder(HardwareMap.DIO_PORT_SHOULDER_ABSOLUTE_ENCODER);
    // Init our settings for our encoder
    initAbsoluteEncoder(absoluteEncoder);

    // Initiatize the settings
    initMotorController(shoulderMotorSlave);
    initMotorController(shoulderMotorMaster);
    // Invert motors (if needed)

    shoulderMotorMaster.setInverted(true);
    shoulderMotorSlave.setInverted(InvertType.OpposeMaster);

    // Have the slave follow the master
    shoulderMotorSlave.follow(shoulderMotorMaster, FollowerType.PercentOutput);

    // Reset the encoders
    resetEncoders(shoulderMotorMaster);
    // resetEncoders(shoulderMotorSlave); Shouldn't need to do on the slave since
    // the slave should always be in follow mode.

    // Get the absolute position of the shoulder at RobotInit
    initialAbsolutePosition = absoluteEncoder.getAbsolutePosition();
    referencePosition = initialAbsolutePosition; // Safe this also in another variable for later

    // Configure Motion Magic on the Motors
    configSimpleMM(shoulderMotorMaster);

    // Add PID Fields to SmartDashboard
    // SmartDashboard.putNumber("Position", 0);
    // SmartDashboard.putNumber("kF", Constants.SHOULDER_DEFAULT_kF);
    // SmartDashboard.putNumber("kP", Constants.SHOULDER_DEFAULT_kP);
    // SmartDashboard.putNumber("kI", Constants.SHOULDER_DEFAULT_kI);
    // SmartDashboard.putNumber("kD", Constants.SHOULDER_DEFAULT_kD);
    // SmartDashboard.putNumber("Cruise Vel", Constants.SHOULDER_MM_VELOCITY);
    // SmartDashboard.putNumber("Cruise Accel ", Constants.SHOULDER_MM_ACCELERATION);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put the encoder value of the Master Motor to the Dashboard
    shoulderMasterPosition = shoulderMotorMaster.getSelectedSensorPosition();
    // shoulderSlavePosition = shoulderMotorSlave.getSelectedSensorPosition();
    // SmartDashboard.putNumber("Shoulder Encoder Position", shoulderMotorMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shoulder Absolute Position", absoluteEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Absolute Encoder Distance",
    // absoluteEncoder.getDistance() * -1);
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
    talon.configNeutralDeadband(Constants.SHOULDER_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS, Constants.kTimeoutMs);
    talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.kTimeoutMs);
    talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.kTimeoutMs);
    /* Set the peak and nominal outputs */
    talon.configNominalOutputForward(Constants.SHOULDER_NOMINAL_OUTPUT_FORWARD, Constants.kTimeoutMs);
    talon.configNominalOutputReverse(Constants.SHOULDER_NOMINAL_OUTPUT_REVERSE, Constants.kTimeoutMs);
    talon.configPeakOutputForward(Constants.SHOULDER_PEAK_OUTPUT_FORWARD, Constants.kTimeoutMs);
    talon.configPeakOutputReverse(Constants.SHOULDER_PEAK_OUTPUT_REVERSE, Constants.kTimeoutMs);
    // Set how many seconds that the motor can ramp from neutral to full
    talon.configClosedloopRamp(Constants.SHOULDER_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS, Constants.kTimeoutMs);
    /**
     * Configure the current limits that will be used
     * Stator Current is the current that passes through the motor stators.
     * Use stator current limits to limit rotor acceleration/heat production
     * Supply Current is the current that passes into the controller from the supply
     * Use supply current limits to prevent breakers from tripping
     *
     * enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
     */
    talon.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, MaxMotorAmpsConstants.MAX_AMPS_STATOR_LIMIT_FALCON500,
            MaxMotorAmpsConstants.MAX_AMPS_STATOR_TRIGGER_FALCON500,
            MaxMotorAmpsConstants.MAX_SECS_STATOR_THRESHOLDTIME_FALCON500));
  }

  /**
   * Resets the encoders based on the absolute encoder reading if available or to 0 if not available
   * @param talon The motor to set the ecoder position of
   */

  public void resetEncoders(WPI_TalonFX talon) {
    double newMotorPosition;
    // Check to see if absolute encoder is present and use it's position if so.
    if (absoluteEncoder.isConnected()) {
      // Get the current distance of the shoulder calculated based off of absolute
      // encoder.
      newMotorPosition = getCurrentAbosoluteDistance() ; // Negating to change phase

      // Check to make sure it's a reasonable number in case the encoder crossed over
      // the 0 line i.e. is reading 0.99
      if (newMotorPosition < (Constants.SHOULDER_POSITION_MIN - 2000) || newMotorPosition > Constants.SHOULDER_POSITION_MAX) {
        System.out.println("Encoder-calculated shoulder position outside of acceptable range. " + newMotorPosition);
        newMotorPosition = 0;
      }
   } else {
    //Absolute encoder not detected so just setting position to 0
    newMotorPosition = 0;
    }
    // Write out the intial position of the shoulder motor
    SmartDashboard.putNumber("Shoulder Init Position:", newMotorPosition);
    // Set the motor encoder to the new position
    talon.getSensorCollection().setIntegratedSensorPosition(newMotorPosition, Constants.kTimeoutMs);
  }

  // Configure our PID Values
  public void configPIDFValues(WPI_TalonFX talon, double p, double i, double d, double f, int slot) {
    // Configure the PID settings for a slot
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
    // Set up PID Values for Slot 0, going up
    configPIDFValues(talon, Constants.SHOULDER_DEFAULT_kP, Constants.SHOULDER_DEFAULT_kI, Constants.SHOULDER_DEFAULT_kD,
        Constants.SHOULDER_DEFAULT_kF, 0);
    // Set up PID Values for Slot 1, going down
    configPIDFValues(talon, Constants.SHOULDER_DEFAULT_kP_slot1, Constants.SHOULDER_DEFAULT_kI_slot1,
        Constants.SHOULDER_DEFAULT_kD_slot1,
        Constants.SHOULDER_DEFAULT_kF_slot1, 1);

    configMotionCruiseAndAcceleration(talon, Constants.SHOULDER_MM_VELOCITY, Constants.SHOULDER_MM_ACCELERATION);
    configAllowableError(talon, 0, Constants.SHOULDER_ALLOWED_ERROR);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
  }

  // Stop our motor(s)
  public void stopMotor() {
    shoulderMotorMaster.stopMotor();
  }

  // Use MotionMagic to set the shoulder to a specific Encoder Position.
  public void setShoulderPosition(double setPoint) {
    // Configure our Vel, AccL, and SLOT whether we are going up or down.
    manageMotion(setPoint);
    shoulderMotorMaster.setSafetyEnabled(false);
    // Added this command to make sure the slave is following the master each and
    // everytime
    // we move the shoulder. (The slave kept unfollowing during testing)
    shoulderMotorSlave.follow(shoulderMotorMaster, FollowerType.PercentOutput);
    // distance = SmartDashboard.getNumber("MM Distance", 1000);
    if (safeToMoveShoulder()) {
      this.setPoint = setPoint;
      shoulderMotorMaster.set(TalonFXControlMode.MotionMagic, setPoint, DemandType.ArbitraryFeedForward,
          calculateArbitraryFF(setPoint));
    } else {
      stopMotor();
    }
  }

  // Return the shoulder's position
  public double getShoulderPosition() {
    return shoulderMasterPosition;
  }

  // Method to test the shoulder with the SmartDashboard and get PID values
  public void testShoulderMM(double setPoint, double kF, double kP, double kI, double kD, double cruiseVel,
      double cruiseAccel) {
    configPIDFValues(shoulderMotorMaster, kP, kI, kD, kF, 0);
    configMotionCruiseAndAcceleration(shoulderMotorMaster, cruiseVel, cruiseAccel);
    shoulderMotorMaster.setSafetyEnabled(false);
    // distance = SmartDashboard.getNumber("MM Distance", 1000);
    this.setPoint = setPoint;
    System.out.println("Setpoint  " + setPoint);
    System.out.println("kF  " + kF);
    System.out.println("kP  " + kP);
    System.out.println("Velocity  " + cruiseVel);
    System.out.println("Acceleration  " + cruiseAccel);
    shoulderMotorMaster.set(TalonFXControlMode.MotionMagic, setPoint);
  }

  // Joystick method to move the winch manually
  public void moveShoulder(double speed) {
    if (Math.abs(speed) > .1) {
      if (safeToMoveShoulder()) {
        shoulderMotorMaster.set(TalonFXControlMode.PercentOutput, speed);
      }
    } else {
      shoulderMotorMaster.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  // Method to check whether we are in a safe range to move the arm
  public boolean safeToMoveShoulder() {
    // Get the current speed of the shoulder
    double shoulderSpeed = shoulderMotorMaster.get();
    // If our position is greater than our Min Pos, and we want to lift the
    // shoulder, then we are good
    if ((shoulderMasterPosition >= Constants.SHOULDER_POSITION_MIN) & (shoulderSpeed >= 0)) {
      return true;
      // If our position is less than our Max Pos, and we want to lower the shoulder,
      // then we are good
    } else if ((shoulderMasterPosition <= Constants.SHOULDER_POSITION_MAX) && (shoulderSpeed <= 0)) {
      return true;
    }
    // We are outside of oru range.. Not safe to move shoulder
    else {
      System.out.println("It is no longer safe to move the shoulder.  Shoulder Position:  " + shoulderMasterPosition);
      return false;
    }
  }

  // Method to check whether we are in a safe range to extend the
  // Extender and flip the wrist
  public boolean safeToExtendAndWrist() {
    if (shoulderMasterPosition <= PositionSetpoints.SHOULDER_POSITION_SAFE_TO_EXTEND) {
      System.out.println(
          "Shoulder is not in position to allow to extend or wrist.  Shoulder Position:  " + shoulderMasterPosition);
      return false;
    } else {
      return true;
    }
  }

  /**
   * Lowers the Shoulder by the SHOULDER_MOVEMENT_INCREMENT
   */
  public void lowerShoulder() {
    // Retracting the wrist is moving it in a negative direction. Need to make sure
    // we
    // are not lower than the minimal position
    double newSetPoint = setPoint - Constants.SHOULDER_MOVEMENT_INCREMENT;
    if (setPointIsValid(newSetPoint)) {
      setPoint = newSetPoint;
      setShoulderPosition(setPoint);
    } else {
      System.out.println("Shoulder Setpoint at it's lower limit allready: " + setPoint);
    }
  }

  /**
   * Raises the Shoulder by the SHOULDER_MOVEMENT_INCREMENT
   */
  public void raiseShoulder() {
    // Lowering the wrist is moving it in a positive direction. Need to make sure we
    // are not higher than the minimal position
    double newSetPoint = setPoint + Constants.SHOULDER_MOVEMENT_INCREMENT;
    if (setPointIsValid(newSetPoint)) {
      setPoint = newSetPoint;
      setShoulderPosition(newSetPoint);
    } else {
      System.out.println("Shoulder Setpoint at it's higher limit allready: " + setPoint);
    }
  }

  /**
   * Will hold last known setpoint
   */
  public void holdPostion() {
    // winchMotorMaster.set(TalonFXControlMode.MotionMagic, setPoint);
  }

  // Return Aribitrary Feed Forward based on trig for the arm
  public double calculateArbitraryFF(double targetPos) {
    double kMeasuredPosHorizontal = Constants.SHOULDER_HORIZONTAL_POS; // Position measured when arm is horizontal
    double kTicksPerDegree = 2048 * 192 / 360; // Enoder is 2489 ticks. 192 = Gear reduction and sprockets
    double currentPos = shoulderMotorMaster.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    double maxGravityFF = Constants.SHOULDER_MAX_GRAVITY_kF;
    return maxGravityFF * cosineScalar;
  }

  /**
   * Checks to see if the provided setPoint within the legal bounds
   * 
   * @param newSetPoint
   * @return true if it falls on or between min and max allowed values.
   */
  private boolean setPointIsValid(double newSetPoint) {
    // System.out.println("New Setpoint: " + newSetPoint + " Current Setpoint: " +
    // this.setPoint);
    // If we are going down, we just have to make sure new setpoint is above our min
    // position
    if ((newSetPoint < this.setPoint) && (newSetPoint >= Constants.SHOULDER_POSITION_MIN)) {
      // System.out.println("Setpoint is valid: " + newSetPoint);
      return true;
    }
    // If we are going up, we just have to make sure new setpoint is below our max
    // position
    else if ((newSetPoint > this.setPoint) && (newSetPoint <= Constants.SHOULDER_POSITION_MAX)) {
      // System.out.println("Setpoint is valid: " + newSetPoint);
      return true;
    } else {
      System.out
          .println(
              "Given position " + newSetPoint + " is outside legal bounderies of " + Constants.SHOULDER_POSITION_MIN
                  + " and " + Constants.SHOULDER_POSITION_MAX);
      return false;
    }
  }

  // Quick method to put the TalonFX in a different Control Mode, just in case
  // MotionMagic goes wonky after two
  // Motion Magic calls in succession.
  public void resetMotionMagic() {
    shoulderMotorMaster.set(0);
  }

  // Just in case we need to make the slave follow the master again
  public void slaveFollowMaster() {
    shoulderMotorSlave.follow(shoulderMotorMaster, FollowerType.PercentOutput);
  }

  // Trying a new method to use two slots on the TalonFX.
  // We used this to smooth out the arm motion due to gravity
  // Slot 0 for up motion
  // Slot 1 for down motion
  public void manageMotion(double targetPosition) {
    // Get our current position
    double currentPosition = shoulderMasterPosition;

    // going up
    if (currentPosition < targetPosition) {
      // set accel and velocity for going up
      shoulderMotorMaster.configMotionAcceleration(Constants.SHOULDER_MM_ACCELERATION);
      shoulderMotorMaster.configMotionCruiseVelocity(Constants.SHOULDER_MM_VELOCITY);
      // Select the Up Gains
      shoulderMotorMaster.selectProfileSlot(0, 0);
    } else {
      // set accel and velocity for going down
      shoulderMotorMaster.configMotionAcceleration(Constants.SHOULDER_MM_ACCELERATION_DOWN);
      shoulderMotorMaster.configMotionCruiseVelocity(Constants.SHOULDER_MM_VELOCITY_DOWN);
      // Select the Down Gains
      shoulderMotorMaster.selectProfileSlot(1, 0);

    }
  }

  // Setup the Through Bore Encoder from REV's Specsheet:
  // https://docs.revrobotics.com/through-bore-encoder/specifications
  private void initAbsoluteEncoder(DutyCycleEncoder encoder) {
    encoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0); // PERIOD = 1025 for the Encoder
    encoder.setDistancePerRotation(2048 * 192); // 2048 talonfx ticks * 192:1 Gear/Sprocket Reduction
    encoder.setPositionOffset(Constants.SHOULDER_ABSOLUTE_ENCODER_OFFSET);
  }

  // public void calibrateEncoders(){
  // // Move the mechanism to the reference position
  // // Read the position of both encoders at the reference position
  // double absolutePosition = absoluteEncoder.get();
  // double relativePosition = shoulderMotorMaster.getSelectedSensorPosition();

  // // Calculate the offset between the absolute and relative encoders at the
  // reference position
  // relativeOffset = absolutePosition - relativePosition;

  // // Adjust the readings of the relative encoder by the offset
  // shoulderMotorMaster.setSelectedSensorPosition(0);
  // //relativeEncoder.setReverseDirection(true);
  // }

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
  public void syncEncoders() {
    resetEncoders(shoulderMotorMaster);
  }

}