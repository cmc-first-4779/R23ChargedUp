// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Open and close gripper 

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  DoubleSolenoid gripper; 
  public Gripper() {
    gripper=new DoubleSolenoid(null, 0,1); 


  }

  // Declaring my gripper as an double solonoid




  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void OpenGripper() {
    gripper.set(DoubleSolenoid.Value.kReverse);
  }

  public void CloseGripper() {
    gripper.set(DoubleSolenoid.Value.kForward);
  }

}
