// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ExtenderCommands.ExtenderStopCommand;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import frc.robot.commands.ShoulderCommands.ShoulderStopCommand;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//  This Command Group stops everything on the robot in case of emergency.

public class StopAllPCG extends ParallelCommandGroup {
  // Declare our Subsystems
  ShoulderSubsystem shoulder;
  ExtenderSubsystem extender;
  IntakeSubsystem intake;

  /** Creates a new StopAllPCG. */
  public StopAllPCG(ShoulderSubsystem shoulder, ExtenderSubsystem extender, IntakeSubsystem intake) {

    this.shoulder = shoulder;
    this.extender = extender;
    this.intake = intake;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShoulderStopCommand(shoulder),
        new ExtenderStopCommand(extender),
        new IntakeStopCommand(intake)

    );
  }
}
