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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.StaticConstants.HardwareMap;
import frc.StaticConstants.MaxMotorAmpsConstants;
import frc.robot.Constants;

/*  This Subsystem is for our Winch that raises and lowers the arm to certain 
 *  set positions.
 * 
 *   We are assuming two Falcon 500 Motors with TalonFX Controllers
 *   Using Motion Magic to drive the arm.
 */

public class WinchSubsystem extends SubsystemBase {

  /** Creates a new ExtenderSubsystem. */
  // Declare our TalonFX Motor Controllers
  WPI_TalonFX winchMotorSlave;
  WPI_TalonFX winchMotorMaster;
  // Declare our Encoders
  RelativeEncoder winchEncoderSlave;
  RelativeEncoder winchEncoderMaster;

  /** Creates a new WinchPulleySubsystem. */
  public WinchSubsystem() {
    // Address our controllers
    winchMotorSlave = new WPI_TalonFX(HardwareMap.CAN_ADDRESS_WINCH_MOTOR_LEFT);
    winchMotorMaster = new WPI_TalonFX(HardwareMap.CAN_ADDRESS_WINCH_MOTOR_RIGHT);
    // Initiatize the settings
    initMotorController(winchMotorSlave);
    initMotorController(winchMotorMaster);
    // Invert motors (if needed)
    winchMotorSlave.setInverted(true);
    winchMotorMaster.setInverted(false);
    // Have the left motor follow the right motor
    winchMotorSlave.follow(winchMotorMaster);
    // Reset the encoders
    resetEncoders(winchMotorMaster);
    resetEncoders(winchMotorSlave);
    // Configure Motion Magic on the Motors
    configSimpleMM(winchMotorMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put the encoder value of the Master Motor to the Dashboard
    SmartDashboard.putNumber("Winch Encoder Position", winchMotorMaster.getSelectedSensorPosition());
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
    talon.configNeutralDeadband(Constants.WINCH_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS, Constants.kTimeoutMs);
    talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.kTimeoutMs);
    talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.kTimeoutMs);
    /* Set the peak and nominal outputs */
    talon.configNominalOutputForward(Constants.WINCH_NOMINAL_OUTPUT_FORWARD, Constants.kTimeoutMs);
    talon.configNominalOutputReverse(Constants.WINCH_NOMINAL_OUTPUT_REVERSE, Constants.kTimeoutMs);
    talon.configPeakOutputForward(Constants.WINCH_PEAK_OUTPUT_FORWARD, Constants.kTimeoutMs);
    talon.configPeakOutputReverse(Constants.WINCH_PEAK_OUTPUT_REVERSE, Constants.kTimeoutMs);
    // Set how many seconds that the motor can ramp from neutral to full
    talon.configClosedloopRamp(Constants.WINCH_CLOSED_LOOP_NEUTRAL_TO_FULL_SECS, Constants.kTimeoutMs);
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

  // Resets our Encoder to ZERO
  public void resetEncoders(WPI_TalonFX talon) {
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

  // After our first event, we ended up deploying the intake arms via Motion
  // Magic.
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

  private void configSimpleMM(WPI_TalonFX talon) {
    // Tell each talon to use Quad Encoder as their PID0
    talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_CLOSED_LOOP,
        Constants.kTimeoutMs);
    // Talons have 4 slots of PID variables and 2 PID indexes. Set the PID0 to use
    // Slot0
    talon.selectProfileSlot(0, 0);
    // Set up PID Values for the Winch
    configPIDFValues(talon, Constants.WINCH_DEFAULT_P, Constants.WINCH_DEFAULT_I, Constants.WINCH_DEFAULT_D,
        Constants.WINCH_DEFAULT_F, 0); // STILL NEED TO GET THESE VALUES
    configMotionCruiseAndAcceleration(talon, Constants.WINCH_MM_VELOCITY, Constants.WINCH_MM_ACCELERATION);
    configAllowableError(talon, 0, 500);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
  }

  // Stop our motor(s)
  public void stopMotor() {
    winchMotorMaster.stopMotor();
  }

  // Use MotionMagic to set the winch to a specific Encoder Position.
  public void setWinchPosition(double setPoint) {
    winchMotorMaster.setSafetyEnabled(false);
    // distance = SmartDashboard.getNumber("MM Distance", 1000);
    winchMotorMaster.set(TalonFXControlMode.MotionMagic, setPoint);
  }

  public void testWinchMM(double setPoint, double kF, double kP, double kI, double kD, double cruiseVel,
      double cruiseAccel) {
    configPIDFValues(winchMotorMaster, kP, kI, kD, kF, 0);
    configMotionCruiseAndAcceleration(winchMotorMaster, cruiseVel, cruiseAccel);
    winchMotorMaster.setSafetyEnabled(false);
    // distance = SmartDashboard.getNumber("MM Distance", 1000);
    winchMotorMaster.set(TalonFXControlMode.MotionMagic, setPoint);
  }

  public void moveWinch(double speed) {
    if (Math.abs(speed) > .1) {
      winchMotorMaster.set(TalonFXControlMode.PercentOutput, speed);
    } else {
      winchMotorMaster.set(TalonFXControlMode.PercentOutput, 0);
    }
  }
}
