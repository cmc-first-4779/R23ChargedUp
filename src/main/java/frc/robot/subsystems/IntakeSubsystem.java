// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StaticConstants.HardwareMap;
import frc.robot.StaticConstants.MaxMotorAmpsConstants;

//  INTAKE SUBSYSTEM:  Used to pickup or eject a game piece

public class IntakeSubsystem extends SubsystemBase {
  // Declare our motor
  CANSparkMax intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Address our Motor Controller
    intakeMotor = new CANSparkMax(HardwareMap.CAN_ADDRESS_INTAKE, MotorType.kBrushless);
    // Initiatize our Motor Controller
    initSparkMaxMotorController(intakeMotor, "NEO550");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Encoder Velocity", intakeMotor.getEncoder().getVelocity());
  }

  // This is a test line for training in Github
  // Initialize a SparkMax Motor controller and set our default settings.
  private static void initSparkMaxMotorController(CANSparkMax sparkMax, String Motortype) {
    System.out.println("Initializing SparkMax: " + sparkMax);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setIdleMode(IdleMode.kBrake); // kCoast is Coast... kBrake is Brake
    if (Motortype == "NEO550") {
      sparkMax.setSmartCurrentLimit(MaxMotorAmpsConstants.MAX_AMPS_STATOR_LIMIT_NEO550); // Set the Amps limit
    } else {
      sparkMax.setSmartCurrentLimit(MaxMotorAmpsConstants.MAX_AMPS_STATOR_LIMIT_NEO); // Set the Amps limit
    }
    // sparkMax.burnFlash(); // Burn these settings into the flash in case of an
    // electrical issue.
  }

  // Stop the Intake motor
  public void stopMotor() {
    intakeMotor.stopMotor();
  }

  // Set the Intake to a specific Speed to pick up something
  public void intakeRun(double speed) {
    intakeMotor.set(speed);
  }

  // Method to check the current of the intake motor.
  // If it spikes above a specific value which the know is similar to the motor
  // under load, we return a
  // "true" boolean
  // We need five readings fo a spiked output current in a row to determine that
  // we have a good signal
  public boolean isIntakeMotorUnderLoad() {
    // Set our "now" current to the output current
    double nowCurrent = intakeMotor.getOutputCurrent();
    // declare boolean flags to hold the memory of the past two cycles
    boolean isIntakeUnderLoad_now = false;
    boolean isIntakeUnderLoad_oneCycleAgo = false;
    boolean isIntakeUnderLoad_twoCyclesAgo = false;
    boolean isIntakeUnderLoad_threeCyclesAgo = false;
    boolean isIntakeUnderLoad_fourCyclesAgo = false;
    // Our Counter starts at 0
    int counter = 0;
    // Start our while loop to check the Output current for a number of cycles
    while (counter <= Constants.INTAKE_CURRENT_CHECK_COUNTER) {
      // Set our boolean to the past few cycles for testing
      isIntakeUnderLoad_fourCyclesAgo = isIntakeUnderLoad_threeCyclesAgo;
      isIntakeUnderLoad_threeCyclesAgo = isIntakeUnderLoad_twoCyclesAgo;
      isIntakeUnderLoad_twoCyclesAgo = isIntakeUnderLoad_oneCycleAgo;
      isIntakeUnderLoad_oneCycleAgo = isIntakeUnderLoad_now;
      // Get our "now" current
      nowCurrent = intakeMotor.getOutputCurrent();
      // check to see if the current is over our limit
      if (nowCurrent >= Constants.INTAKE_CURRENT_WITH_CUBE) {
        // Set the "now" flag to true
        isIntakeUnderLoad_now = true;
      } else {
        // Set the "now" flag to false
        isIntakeUnderLoad_now = false;
      }
      // increment our counter
      counter++;
    }
    // if all of our isIntakeUnderLoad flags are true, then it's a good signal
    if ((isIntakeUnderLoad_fourCyclesAgo == true) && (isIntakeUnderLoad_threeCyclesAgo == true)
        && (isIntakeUnderLoad_twoCyclesAgo == true) && (isIntakeUnderLoad_oneCycleAgo == true)
        && (isIntakeUnderLoad_now == true)) {
      return true;
    } else {
      return false;
    }

  }

}
