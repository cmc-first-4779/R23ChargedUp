// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PositionSetpoints;
import frc.robot.commands.ExtenderCommands.ExtenderSetPosition;
//import frc.robot.commands.MiscCommands.SyncEncoders;
import frc.robot.commands.MiscCommands.Wait;
import frc.robot.commands.ShoulderCommands.ResetShoulderMM;
import frc.robot.commands.ShoulderCommands.ShoulderSetPosition;
import frc.robot.commands.WristCommands.WristSetPosition;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SafeRectractToStowSCG extends SequentialCommandGroup {
  // Declare our subsystems
  ShoulderSubsystem shoulder;
  ExtenderSubsystem extender;
  WristSubsystem wrist;

  /** Creates a new SafeRectractToStowSCG. */
  public SafeRectractToStowSCG(ShoulderSubsystem shoulder, ExtenderSubsystem extender, WristSubsystem wrist) {
    // Set our local variables to our passthrough variables
    this.shoulder = shoulder;
    this.extender = extender;
    this.wrist = wrist;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Retract the wrist last to avoid stripping off parts
        new WristSetPosition(wrist, PositionSetpoints.WRIST_POSITION_STOW),
        // new ShoulderSetPosition(shoulder, 35000),
        new ExtenderSetPosition(extender, PositionSetpoints.EXTENDER_POSITION_STOW),
        // Wait a little time for the system to settle down (seconds)
        new Wait(0.45),
        // Retract the Shoulder, Extender, and Waist
        new RetractShoulderExtenderPCG(shoulder, extender),
        // new ExtenderSetPosition(extender, PositionSetpoints.EXTENDER_POSITION_STOW),
        // new Wait(0.45),
        // new ShoulderSetPosition(shoulder,
        // PositionSetpoints.SHOULDER_POSITION_SAFE_TO_EXTEND),
        // Wait a little time for the system to settle down (seconds)
        // new Wait(0.45),

        // Disengage Motion Magic to prevent wonkiness from two successive calls.
        new ResetShoulderMM(shoulder),
        // Put the shoulder in the STOW position
        new ShoulderSetPosition(shoulder, PositionSetpoints.SHOULDER_POSITION_STOW));
        //new Wait(0.15),
        //new SyncEncoders(shoulder, wrist));
  }
}
