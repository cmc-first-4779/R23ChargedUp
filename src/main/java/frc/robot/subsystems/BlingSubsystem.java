// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StaticConstants.HardwareMapConstants;

public class BlingSubsystem extends SubsystemBase {
  /** Creates a new BlingSubsystem. */

  // Declare our Bling Controller as a Spark. (Same libraries from REV Robotics)
  Spark blingController;

  public BlingSubsystem() {
    // Initiate the blingController object
    blingController = new Spark(HardwareMapConstants.PWM_PORT_BLING);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Set the pattern for Bling to whatever pattern we pass it.
  // * All of our patterns are in the StaticConstants folder under BlingConstants
  public void setBlingPattern(double pattern) {
    blingController.set(pattern);
  }
}