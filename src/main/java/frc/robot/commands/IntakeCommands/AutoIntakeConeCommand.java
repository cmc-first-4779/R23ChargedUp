// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.MiscCommands.Wait;
import frc.robot.subsystems.IntakeSubsystem;

//  This Command Group embeds a Timer into the IntakeCone Command so that we can use 
//    the command in Auton so that it will end.
//   (NOTE:  THIS IS A PARALLEL RACE GROUP)


public class AutoIntakeConeCommand extends ParallelRaceGroup {
  IntakeSubsystem intake;
  /** Creates a new AutoIntakeConeCommand. */
  public AutoIntakeConeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeConeCommand(intake),
      new Wait(Constants.INTAKE_CONE_AUTON_TIMER)
    );
  }
}
