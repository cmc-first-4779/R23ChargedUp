// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.StaticConstants.MaxMotorAmpsConstants;
import frc.robot.Constants;
import frc.robot.StaticConstants.HardwareMapConstants;

public class WristSubsystem extends SubsystemBase {
  WPI_TalonFX wristMotorLeft;
  WPI_TalonFX wristMotorRight;
  RelativeEncoder wristEncoderLeft;
  RelativeEncoder wristEncoderRight;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    wristMotorLeft = new WPI_TalonFX(HardwareMapConstants.CAN_ADDRESS_WRIST_MOTOR_LEFT);
    wristMotorRight = new WPI_TalonFX(HardwareMapConstants.CAN_ADDRESS_WRIST_MOTOR_RIGHT);
    initMotorController(wristMotorLeft);
    initMotorController(wristMotorRight);
    wristMotorLeft.setInverted(true);
    wristMotorRight.setInverted(false);
    initMM(wristMotorLeft);
    initMM(wristMotorRight);
    resetEncoders();
    zeroEncoders();
    // Set up PID Values for the Climber
    configPIDFValues(Constants.WRIST_DEFAULT_P, Constants.WRIST_DEFAULT_I, Constants.WRIST_DEFAULT_D,
        Constants.WRIST_DEFAULT_F, 0); // STILL NEED TO GET THESE VALUES
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Initialize a TalonFX Motor controller and set our default settings.
  private static void initMotorController(WPI_TalonFX talon) {
    System.out.println("Initializing Falcon: " + talon);
    talon.configFactoryDefault();
    talon.setNeutralMode(NeutralMode.Brake); // Neutral Mode is Brake
    talon.neutralOutput();
    talon.setSensorPhase(false);
    talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    talon.configNominalOutputForward(0.0, 0);
    talon.configNominalOutputReverse(0.0, 0);
    talon.configClosedloopRamp(0.5, 0);
    /**
     * Configure the current limits that will be used
     * Stator Current is the current that passes through the motor stators.
     * Use stator current limits to limit rotor acceleration/heat production
     * Supply Current is the current that passes into the controller from the supply
     * Use supply current limits to prevent breakers from tripping
     *
     * enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
     */
    // talon.configStatorCurrentLimit(MaxMotorAmpsConstants.MAX_AMPS_STATOR_LIMIT_FALCON500);
    talon.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, MaxMotorAmpsConstants.MAX_AMPS_STATOR_LIMIT_FALCON500,
            MaxMotorAmpsConstants.MAX_AMPS_STATOR_TRIGGER_FALCON500,
            MaxMotorAmpsConstants.MAX_SECS_STATOR_THRESHOLDTIME_FALCON500));

  }

  public void initMM(WPI_TalonFX talon) {
    talon.configMotionCruiseVelocity(Constants.WRIST_MM_VELOCITY);
    talon.configMotionCruiseVelocity(Constants.WRIST_MM_ACCELERATION);
  }

  // Method to reset our encoders
  public void resetEncoders() {
    wristMotorLeft.setSelectedSensorPosition(0);
    wristMotorRight.setSelectedSensorPosition(0);
  }

  // Zero's out both encoders
  public void zeroEncoders() {
    int kTimeoutMs = 30;
    wristMotorLeft.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    wristMotorRight.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    // System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
  }

  // Set up our PID Values for the motor controllers
  public void configPIDFValues(double p, double i, double d, double f, int slot) {
    // Configure the PID settings for Slot0
    wristMotorLeft.config_kF(slot, f);
    wristMotorLeft.config_kP(slot, p);
    wristMotorLeft.config_kI(slot, i);
    wristMotorLeft.config_kD(slot, d);
    wristMotorRight.config_kF(slot, f);
    wristMotorRight.config_kP(slot, p);
    wristMotorRight.config_kI(slot, i);
    wristMotorRight.config_kD(slot, d);
  }

  // Method to stop our motors
  public void stopMotors() {
    wristMotorLeft.stopMotor();
    wristMotorRight.stopMotor();
  }
// This sets the target Climber position for the controllers based off of
  // Constant
  public void setPosition(double position) {
    wristMotorLeft.set(TalonFXControlMode.Position, position);
    wristMotorRight.set(TalonFXControlMode.Position, position);
  }

// Control the Up and Down Values with Motion Magic
  public void moveWristMM(double ySpeed) {
    wristMotorLeft.set(TalonFXControlMode.MotionMagic, ySpeed);
    wristMotorRight.set(TalonFXControlMode.MotionMagic,ySpeed);
  }
  

}
