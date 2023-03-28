// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StaticConstants.HardwareMap;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

//  BLING SUBSYSTEM:  The LEDs on our robot.  This year these will be used to tell
//                     the Human Players which game piece we need.

public class BlingSubsystem extends SubsystemBase {
  /** Creates a new BlingSubsystem. */

  // Declare our Bling Controller as a Spark. (Same libraries from REV Robotics)
  Spark blingController;
  Spark blingController2;  // Just in case we hook up a second REV Blinkin'

  //  Declare the variable for our pattern
  double pattern;

  public BlingSubsystem() {
    // Initiate the blingController object
    blingController = new Spark(HardwareMap.PWM_PORT_BLING);
    blingController2 = new Spark(HardwareMap.PWM_PORT_BLING2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // Set the pattern for Bling to whatever pattern we pass it.
  // * All of our patterns are in the StaticConstants folder under BlingConstants
  public void setBlingPattern(double pattern) {
    blingController.set(pattern);
    blingController2.set(pattern);
  }

  //  Return the current Bling pattern
  public double getBlingPattern(){
    return blingController.get();
  }
}