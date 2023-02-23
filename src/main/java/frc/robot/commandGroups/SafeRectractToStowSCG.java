// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MiscCommands.Wait;
import frc.robot.commands.ShoulderCommands.ShoulderSetPosition;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SafeRectractToStowSCG extends SequentialCommandGroup {
  // Declare our subsystems
  ShoulderSubsystem shoulder;
  ExtenderSubsystem extender;

  /** Creates a new SafeRectractToStowSCG. */
  public SafeRectractToStowSCG(ShoulderSubsystem shoulder, ExtenderSubsystem extender) {
    //  Set our local variables to our passthrough variables
    this.shoulder = shoulder;
    this.extender = extender;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //  Retract the Shoulder, Extender, and Waist
      new RetractShoulderExtenderPCG(shoulder, extender),
      //  Wait a little time for the system to settle down (seconds)
      new Wait(0.5),
      //  Put the shoulder in the STOW position
      new ShoulderSetPosition(shoulder, Constants.SHOULDER_POSITION_STOW)
    );
  }
}
