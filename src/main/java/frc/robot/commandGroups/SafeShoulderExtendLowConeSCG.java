// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ShoulderCommands.ShoulderSetPosition;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SafeShoulderExtendLowConeSCG extends SequentialCommandGroup {
  ShoulderSubsystem shoulder;
  ExtenderSubsystem extender;
  /** Creates a new SafeShoulderExtendLowConeSCG. */
  public SafeShoulderExtendLowConeSCG(ShoulderSubsystem shoulder, ExtenderSubsystem extender) {
    this.shoulder = shoulder;
    this.extender = extender;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(      
    new ShoulderSetPosition(shoulder, Constants.SHOULDER_POSITION_SAFE_TO_EXTEND),
    new ShoulderExtenderLowConePCG(shoulder, extender));
  }
}
