// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PositionSetpoints;
import frc.robot.commands.MiscCommands.Wait;
import frc.robot.commands.ShoulderCommands.ResetShoulderMM;
import frc.robot.commands.ShoulderCommands.ShoulderSetPosition;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSafeSetToPositionSCG extends SequentialCommandGroup {

  // Declare our Subsystem
  ShoulderSubsystem shoulder;
  ExtenderSubsystem extender;
  WristSubsystem wrist;

  // Declare our position
  String position;

  /** Creates a new SafeSetToPositionSCG. */
  public AutoSafeSetToPositionSCG(String position, ShoulderSubsystem shoulder, ExtenderSubsystem extender,
      WristSubsystem wrist) {

    // Set our local variables to the pass-thru values
    this.shoulder = shoulder;
    this.extender = extender;
    this.wrist = wrist;
    this.position = position;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Move the shoulder up to the safe position to move the extender and the wrist
        new ShoulderSetPosition(shoulder, PositionSetpoints.SHOULDER_POSITION_SAFE_TO_EXTEND),
        // Wait for the system to settle down (seconds)
        new Wait(1.0),
        // Disengage Motion Magic to prevent wonkiness from two successive calls.
        new ResetShoulderMM(shoulder),
        // Wait for the system to settle down (seconds)
        new Wait(0.25),
        // Move the shoulder, extender, and wrist the rest of the way to the setpoints
        new AutoSetToPositionPCG(position, shoulder, extender, wrist));
  }
}
