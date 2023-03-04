// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.PositionValues;
import frc.robot.commands.ExtenderCommands.ExtenderSetPosition;
import frc.robot.commands.ShoulderCommands.ShoulderSetPosition;
import frc.robot.commands.WristCommands.WristSetPosition;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetToPositionPCG extends ParallelCommandGroup {

  // Declare our Subsystems
  ShoulderSubsystem shoulder;
  ExtenderSubsystem extender;
  WristSubsystem wrist;

  // position is how we are positioning our subsystems
  String position;

  // Declare our positions
  double shoulderPosition, extenderPosition, wristPosition;

  /** Creates a new SetToPositionPCG. */
  public SetToPositionPCG(String position, ShoulderSubsystem shoulder, ExtenderSubsystem extender, WristSubsystem wrist) {

    // Set local variables to pass-thru variables
    this.shoulder = shoulder;
    this.extender = extender;
    this.wrist = wrist;
    this.position = position;

    // Set up a switch statement to set our position values to the appropriate
    // constants
    // Options for position are: "HIGH CUBE", "HIGH CONE", "MID CUBE", "MID CONE",
    // "LOW CUBE", "LOW CONE", "GROUND", and "STOW"
    switch (position) {
      // Set the values for the Shoulder, Extender, and Wrist Positions to HIGH CUBE
      case "HIGH_CUBE":
        shoulderPosition = PositionValues.SHOULDER_POSITION_HIGH_CUBE_NODE;
        extenderPosition = PositionValues.EXTENDER_POSITION_HIGH_CUBE_NODE;
        wristPosition = PositionValues.WRIST_POSITION_HIGH_CUBE_NODE;
        break;
      // Set the values for the Shoulder, Extender, and Wrist Positions to HIGH CONE
      case "HIGH_CONE":
        shoulderPosition = PositionValues.SHOULDER_POSITION_HIGH_CONE_NODE;
        extenderPosition = PositionValues.EXTENDER_POSITION_HIGH_CONE_NODE;
        wristPosition = PositionValues.WRIST_POSITION_HIGH_CONE_NODE;
        break;
      // Set the values for the Shoulder, Extender, and Wrist Positions to MID CUBE
      case "MID_CUBE":
        shoulderPosition = PositionValues.SHOULDER_POSITION_MID_CUBE_NODE;
        extenderPosition = PositionValues.EXTENDER_POSITION_MID_CUBE_NODE;
        wristPosition = PositionValues.WRIST_POSITION_MID_CUBE_NODE;
        break;
      // Set the values for the Shoulder, Extender, and Wrist Positions to MID CONE
      case "MID_CONE":
        shoulderPosition = PositionValues.SHOULDER_POSITION_MID_CONE_NODE;
        extenderPosition = PositionValues.EXTENDER_POSITION_MID_CONE_NODE;
        wristPosition = PositionValues.WRIST_POSITION_MID_CONE_NODE;
        break;
      // Set the values for the Shoulder, Extender, and Wrist Positions to LOW CUBE
      case "LOW_CUBE":
        shoulderPosition = PositionValues.SHOULDER_POSITION_LOW_CUBE_NODE;
        extenderPosition = PositionValues.EXTENDER_POSITION_LOW_CUBE_NODE;
        wristPosition = PositionValues.WRIST_POSITION_LOW_CUBE_NODE;
        // Need to set wrist position
        break;
      case "LOW_CONE":
        // Set the values for the Shoulder, Extender, and Wrist Positions to LOW CONE
        shoulderPosition = PositionValues.SHOULDER_POSITION_LOW_CONE_NODE;
        extenderPosition = PositionValues.EXTENDER_POSITION_LOW_CONE_NODE;
        wristPosition = PositionValues.WRIST_POSITION_LOW_CONE_NODE;
        break;
      // Set the values for the Shoulder, Extender, and Wrist Positions to Pick a CUBE
      // from the ground
      case "PICKUP_CUBE":
        shoulderPosition = PositionValues.SHOULDER_POSITION_PICKUP_CUBE;
        extenderPosition = PositionValues.EXTENDER_POSITION_PICKUP_CUBE;
        wristPosition = PositionValues.WRIST_POSITION_PICKUP_CUBE;
        break;
      // Set the values for the Shoulder, Extender, and Wrist Positions to Pick a CONE
      // from the ground
      case "PICKUP_CONE":
        shoulderPosition = PositionValues.SHOULDER_POSITION_PICKUP_CONE;
        extenderPosition = PositionValues.EXTENDER_POSITION_PICKUP_CONE;
        wristPosition = PositionValues.WRIST_POSITION_PICKUP_CONE;
        break;
      case "HUMAN_PLAYER_STATION":
        shoulderPosition = PositionValues.SHOULDER_POSITION_HUMAN_PLAYER_STATION;
        extenderPosition = PositionValues.EXTENDER_POSITION_HUMAN_PLAYER_STATION;
        wristPosition = PositionValues.WRIST_POSITION_HUMAN_PLAYER_STATION;
        break;
      // Our DEFAULT POSITION WILL BE THE STOW POSITION
      default: // DEFAULT position is "STOW"
        shoulderPosition = PositionValues.SHOULDER_POSITION_STOW;
        extenderPosition = PositionValues.EXTENDER_POSITION_STOW;
        wristPosition = PositionValues.WRIST_POSITION_STOW;
        break;
    }

    // Now that we set our shoulder, extender, and wrist positions to the right
    // constant values
    // let's call the Parrallel Command Group for the three subsystem set positions.

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShoulderSetPosition(shoulder, shoulderPosition),
        new ExtenderSetPosition(extender, extenderPosition),
        new WristSetPosition(wrist, wristPosition)
    );
  }
}
