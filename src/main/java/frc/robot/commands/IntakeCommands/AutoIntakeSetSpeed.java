// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.MiscCommands.Wait;
import frc.robot.subsystems.IntakeSubsystem;

//  This Command Group embeds a Timer into the IntakeSetSpeed Command so that we can use 
//    the command in Auton so that it will end.
//   (NOTE:  THIS IS A PARALLEL RACE GROUP)
public class AutoIntakeSetSpeed extends ParallelRaceGroup {
  IntakeSubsystem intake;
  String intakeMode;
  double timer;
  /** Creates a new AutoIntakeSetSpeed. */
  public AutoIntakeSetSpeed(IntakeSubsystem intake, String intakeMode) {
    //  Set our local variables to the pass-thru
    this.intake = intake;
    this.intakeMode = intakeMode;
    
    // Set up a switch statement to set our timer value to the appropriate
    // constant based on the intakeMode that is passed in.
    // (We are doing this to simplify our commands)
    switch (intakeMode) {
      // Set the value for the Eject Cone
      case "EJECT_CONE":
        timer = Constants.EJECT_CONE_AUTON_TIMER;
        break;
      // Set the value for the Eject Cube
      case "EJECT_CUBE":
        timer = Constants.EJECT_CUBE_AUTON_TIMER;
        break;
      // Set the value for the Intake Cone
      case "INTAKE_CONE":
        timer = Constants.INTAKE_CONE_AUTON_TIMER;
        break;
      // Stop the Intake
      case "INTAKE_STOP":
        timer = 0.1;  //  Make this one really short
        break;
      // Default mode is Intake a Cube
      default:
        timer = Constants.INTAKE_CUBE_AUTON_TIMER;
        break;
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeSetSpeed(intake, intakeMode),
      new Wait(timer)
    );
  }
}
