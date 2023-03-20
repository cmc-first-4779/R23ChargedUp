// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StaticConstants.HardwareMap;
import frc.robot.StaticConstants.MaxMotorAmpsConstants;

//  INTAKE SUBSYSTEM:  Used to pickup or eject a game piece

public class IntakeSubsystem extends SubsystemBase {
  // Declare our motor
  CANSparkMax intakeMotor;
    // Declare a variable for filering our outputcurrent
    LinearFilter currentFilter = LinearFilter.movingAverage(Constants.INTAKE_FILTER_TAPS);
    double filteredCurrent, outputcurrent;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Address our Motor Controller
    intakeMotor = new CANSparkMax(HardwareMap.CAN_ADDRESS_INTAKE, MotorType.kBrushless);
    // Initiatize our Motor Controller
    initSparkMaxMotorController(intakeMotor, "NEO550");
  }

  @Override
  public void periodic() {
    outputcurrent = getCurrent();
    // Filter our Current to clean out the noisy signals.
    filteredCurrent = currentFilter.calculate(outputcurrent);
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Intake Encoder Velocity", intakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake Filtered Current", filteredCurrent);
    SmartDashboard.putNumber("Intake Current", outputcurrent);
    System.out.println("Intake Filtered Current:  " +filteredCurrent);
    System.out.println("Intake Output Current:  " +outputcurrent);
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

  //  Method to check the current of the intake motor.
  //  If it spikes above a specific value which the know is similar to the motor under load, we return a 
  //   "true" boolean 
  // public boolean isIntakeMotorUnderLoad() {
  //   double current = intakeMotor.getOutputCurrent();
  //   if (current >= Constants.INTAKE_CURRENT_WITH_LOAD) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  // Method to get the Intake Motor Output Current
  public double getCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public double getOutput() {
    return intakeMotor.getAppliedOutput();
  }

  public void intakeCubeDebounce() {
    // Declare our debouncer
    Debouncer debounce = new Debouncer(Constants.INTAKE_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);
    // While our filtered current is less than the stall current with the cube
    while (debounce.calculate(getFilteredCurrent() < Constants.INTAKE_CURRENT_WITH_CUBE)) {
      // Turn our intake on to take in cubes
      intakeRun(Constants.INTAKE_CUBE_SPEED);
    }
    // Stop the intake motor
    stopMotor();
  }

  public void intakeConeDebounce() {
    // Declare our debouncer and set it to look for a rising spike for a period of time
    Debouncer debounce = new Debouncer(Constants.INTAKE_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);
    // While our filtered current is less than the stall current with the cube
    while (debounce.calculate(getFilteredCurrent() < Constants.INTAKE_CURRENT_WITH_CONE)) {
      // Turn our intake on to take in cubes
      intakeRun(Constants.INTAKE_CONE_SPEED);
    }
    // Stop the intake motor
    stopMotor();
  }

}
