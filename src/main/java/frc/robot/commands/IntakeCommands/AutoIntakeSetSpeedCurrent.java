// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

//  This Command Can be used to intake a Cone or a Cube.
//  The Command automatically ends when the intake Motor Controller
//   detects a current spike, indicating that the intake motor is under load
//   and has either taken in a game piece or ejected one.

public class AutoIntakeSetSpeedCurrent extends CommandBase {
  IntakeSubsystem intake;
  String intakeMode;
  double intakeSpeed;

  /** Creates a new AutoIntakeSetSpeedCurrent. */
  public AutoIntakeSetSpeedCurrent(IntakeSubsystem intake, String intakeMode) {
    // Set our local variables to the pass-thru
    this.intake = intake;
    this.intakeMode = intakeMode;

    // Set up a switch statement to set our speed value to the appropriate
    // constant based on the intakeMode that is passed in.
    // (We are doing this to simplify our commands)
    switch (intakeMode) {
      // Set the value for the Eject Cone
      case "EJECT_CONE":
        intakeSpeed = Constants.EJECT_CONE_SPEED;
        break;
      // Set the value for the Eject Cube
      case "EJECT_CUBE":
        intakeSpeed = Constants.EJECT_CUBE_SPEED;
        break;
      // Set the value for the Intake Cone
      case "INTAKE_CONE":
        intakeSpeed = Constants.INTAKE_CONE_SPEED;
        break;
      // Stop the Intake
      case "INTAKE_STOP":
        intakeSpeed = 0;
        break;
      // Default mode is Intake a Cube
      default:
        intakeSpeed = Constants.INTAKE_CUBE_SPEED;
        break;
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Run the intake at the right speed
    intake.intakeRun(intakeSpeed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return True if we are under Load.. False if we are not..
    return intake.isIntakeMotorUnderLoad(Constants.INTAKE_CURRENT_NUM_HIGH_SIGNALS_NEEDED,
        Constants.INTAKE_CURRENT_NUM_TOTAL_SIGNALS_READ);
  }
}
