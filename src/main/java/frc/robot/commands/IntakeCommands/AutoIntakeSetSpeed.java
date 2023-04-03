// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeSetSpeed extends CommandBase {
  IntakeSubsystem intake;
  String intakeMode;
  // Declare our timer
  Timer timer = new Timer();
  // Declare local variable for keeping track of the time (seconds)
  private double intakeRunTime;
  // Declare our Intake Set Speed
  double intakeSpeed;

  /** Creates a new AutoIntakeSetSpeed2. */
  public AutoIntakeSetSpeed(IntakeSubsystem intake, String intakeMode) {
    // Set our local variables to the pass-thru
    this.intake = intake;
    this.intakeMode = intakeMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set up a switch statement to set our timer value to the appropriate
    // constant based on the intakeMode that is passed in.
    // (We are doing this to simplify our commands)
    switch (intakeMode) {
      // Set the value for the Eject Cone
      case "EJECT_CONE":
        intakeRunTime = Constants.EJECT_CONE_AUTON_TIMER;
        intakeSpeed = Constants.EJECT_CONE_SPEED;
        break;
      // Set the value for the Eject Cube
      case "EJECT_CUBE":
        intakeRunTime = Constants.EJECT_CUBE_AUTON_TIMER;
        intakeSpeed = Constants.EJECT_CUBE_SPEED;
        break;
      // Set the value for the Eject Cube
      case "EJECT_CUBE_AUTON1":
        intakeRunTime = Constants.EJECT_CUBE_AUTON_TIMER;
        intakeSpeed = Constants.EJECT_CUBE_SPEED_AUTON1;
        break;
      // Set the value for the Eject Cube
      case "EJECT_CUBE_STOW":
        intakeRunTime = Constants.EJECT_CUBE_AUTON_TIMER;
        intakeSpeed = Constants.EJECT_CUBE_SPEED_FROMSTOW;
        break;
      // Set the value for the Intake Cone
      case "INTAKE_CONE":
        intakeRunTime = Constants.INTAKE_CONE_AUTON_TIMER;
        intakeSpeed = Constants.INTAKE_CONE_SPEED;
        break;
      // Stop the Intake
      case "INTAKE_STOP":
        intakeRunTime = 0.1; // Make this one really short
        intakeSpeed = 0;
        break;
      // Default mode is Intake a Cube
      default:
        intakeRunTime = Constants.INTAKE_CUBE_AUTON_TIMER;
        intakeSpeed = Constants.INTAKE_CUBE_SPEED;
        break;
    }
    // Run our Intake
    intake.intakeRun(intakeSpeed);
    // Timer reset and start
    timer.reset(); // Reset the timer.
    timer.start(); // Start the timer.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop(); // Stop the timer
    intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If our timer is less than the time we want it to run, keep running the
    // command.
    if (timer.get() < intakeRunTime) {
      return false;
    }
    // Else once we hit our alloted time, exit out of the command.
    else
      return true;
  }
}
