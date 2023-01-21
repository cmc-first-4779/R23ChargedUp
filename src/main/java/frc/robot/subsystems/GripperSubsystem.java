// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.naming.InitialContext;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StaticConstants.HardwareMapConstants;
import frc.robot.StaticConstants.MotorAmpConstants;

public class GripperSubsystem extends SubsystemBase {

  CANSparkMax gripperMotor;
  RelativeEncoder gripperEncoder;

  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    gripperMotor = new CANSparkMax(HardwareMapConstants.CAN_ADDRESS_GRIPPER, MotorType.kBrushless);
    initMotorController(gripperMotor);
    gripperEncoder = gripperMotor.getEncoder();
    gripperEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private double getEncoder() {
    return gripperEncoder.getPosition();

  }

  // Initialize a SparkMax Motor controller and set our default settings.
  private static void initMotorController(CANSparkMax sparkMax) {
    System.out.println("Initializing SparkMax: " + sparkMax);
    sparkMax.restoreFactoryDefaults(); // Reset to the our factory defaults
    sparkMax.setIdleMode(IdleMode.kBrake); // kCoast is Coast... kBrake is Brake
    sparkMax.setSmartCurrentLimit(MotorAmpConstants.MAX_AMPS_STATOR_LIMIT_NEO550); // Set the Amps limit to the
                                                                                   // NEO550 limit
    sparkMax.burnFlash(); // Burn these settings into the flash in case of an electrical issue.
  }

  private void openGripper(double speed) {
    gripperMotor.set(speed);
  }

  private void closeGripper(double speed) {
    gripperMotor.set(speed * -1);

  }

  /**
   * @param speed
   */
  public void stopGripper() {
    gripperMotor.stopMotor();
  }

}
