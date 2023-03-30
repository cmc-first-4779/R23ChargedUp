// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoSense extends CommandBase {
  IntakeSubsystem intake;
  //  Declare our filter for calcualating our average
  LinearFilter currentFilter;
  // Declare our debouncer
  Debouncer debounce;
  //  Declare our current variables
  double filteredCurrent, outputcurrent;
  //  Declare our mode
  String mode;
  //  Declare our stall current variable
  double stallCurrent;
  //  Declare our run speed
  double speed;

  /** Creates a new IntakeAutoSense. */
  public IntakeAutoSense(IntakeSubsystem intake, String mode) {
    this.intake = intake;
    this.mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Declare a variable for filering our outputcurrent
    currentFilter = LinearFilter.movingAverage(Constants.INTAKE_FILTER_TAPS);
    //Get our output current
    outputcurrent = intake.getCurrent();
    // Set our debounce to look for a rising spike for a period of time
    debounce = new Debouncer(Constants.INTAKE_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);
    //  Set up our constants based on what we are trying to pick up
    if (mode == "CONE"){  //Intaking up a CONE
      stallCurrent = Constants.INTAKE_CURRENT_WITH_CONE;
      speed = Constants.INTAKE_CONE_SPEED;
    }
    else{  //  Intaking up a CUBE
      stallCurrent = Constants.INTAKE_CURRENT_WITH_CUBE;
      speed = Constants.INTAKE_CUBE_SPEED;
    }
    // Set the intake speed
    intake.intakeRun(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //  Get the output current for this cycle from the Intake Motor
    outputcurrent = intake.getCurrent();
    // Filter our Current to clean out the noisy signals.
    filteredCurrent = currentFilter.calculate(outputcurrent);
    //  Output our values to the SmartDashboard
    //SmartDashboard.putNumber("Intake Filtered Current", filteredCurrent);
    //SmartDashboard.putNumber("Intake Current", outputcurrent);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //  Stop the Motor when the command ends
    intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //  End the command when our debounce current shows a stall.  Something is in the intake
    return (debounce.calculate(getFilteredCurrent() >= stallCurrent));
  }

  //  Return the filtered current
  public double getFilteredCurrent() {
    return filteredCurrent;
  }
}
