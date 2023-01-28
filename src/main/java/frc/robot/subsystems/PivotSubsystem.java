// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new ExtenderSubsystem. */
  WPI_TalonFX pivotMotor;
  RelativeEncoder pivotEncoder;

  private double m_desiredPosition;
  private boolean IsArmInPosition;

  public PivotSubsystem() {
    pivotMotor = new WPI_TalonFX(HardwareMapConstants.CAN_ADDRESS_EXTENDER);
    initMotorController(pivotMotor);
    pivotMotor.setInverted(true);

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

  // Set up our PID Values for the motor controller
  private void configPIDFValues(double p, double i, double d, double f, int slot) {
    // Configure the PID settings for Slot0
    pivotMotor.config_kF(slot, f);
    pivotMotor.config_kP(slot, p);
    pivotMotor.config_kI(slot, i);
    pivotMotor.config_kD(slot, d);
  }

  public void setStartPosition(Object ControlMode) {
    m_desiredPosition = Constants.PIVOT_START_POSITION;
    configPIDFValues(Constants.PIVOT_kP, Constants.PIVOT_kI, Constants.PIVOT_kD, Constants.PIVOT_kF, 0);
    pivotMotor.set(TalonFXControlMode.Position, m_desiredPosition);
  }

  public void setLowPosition(Object ControlMode) {
    m_desiredPosition = Constants.PIVOT_LOW_POSITION;
    configPIDFValues(Constants.PIVOT_kP, Constants.PIVOT_kI, Constants.PIVOT_kD, Constants.PIVOT_kF, 0);
    pivotMotor.set(TalonFXControlMode.Position, m_desiredPosition);
  }


public void setMidPosition(Object ControlMode) {
    m_desiredPosition = Constants.PIVOT_MID_POSITION;
    configPIDFValues(Constants.PIVOT_kP, Constants.PIVOT_kI, Constants.PIVOT_kD, Constants.PIVOT_kF, 0);
    pivotMotor.set(TalonFXControlMode.Position, m_desiredPosition);
  }

  public void setHighPosition(Object ControlMode) {
    m_desiredPosition = Constants.PIVOT_HIGH_POSITION;
    configPIDFValues(Constants.PIVOT_kP, Constants.PIVOT_kI, Constants.PIVOT_kD, Constants.PIVOT_kF, 0);
    pivotMotor.set(TalonFXControlMode.Position, m_desiredPosition);
  }
  
  public void stopMotor() {
    pivotMotor.stopMotor();
  }

  public double getPivotPosition() {
    return pivotMotor.getSelectedSensorPosition();

  }

  public void resetEncoder() {
    pivotMotor.setSelectedSensorPosition(0);
  }

  public void rotatePivotUp() {
    pivotMotor.set(Constants.PIVOT_SPEED);
  }

  public void rotatePivotDown() {
    pivotMotor.set(Constants.PIVOT_SPEED * -1);
  }

}
