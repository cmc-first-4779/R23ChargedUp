// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.StaticConstants.MaxMotorAmpsConstants;
import frc.robot.Constants;
import frc.robot.StaticConstants.HardwareMapConstants;

public class ExtenderSubsystem extends SubsystemBase {
  /** Creates a new ExtenderSubsystem. */
  WPI_TalonFX extenderMotor;
  RelativeEncoder extenderMotorEncoder;

  public ExtenderSubsystem() {
    extenderMotor = new WPI_TalonFX(HardwareMapConstants.CAN_ADDRESS_EXTENDER);
    initMotorController(extenderMotor);
    extenderMotor.setInverted(true);

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

  public void stopMotor() {
    extenderMotor.stopMotor();
  }

  public double getExtenderPosition() {
    return extenderMotor.getSelectedSensorPosition();

  }

  public void resetEncoder() {
    extenderMotor.setSelectedSensorPosition(0);
  }

  public void extendArm() {
    extenderMotor.set(Constants.EXTENDER_SPEED);
  }

  public void retractArm() {
    extenderMotor.set(Constants.EXTENDER_SPEED * -1);
  }

}
