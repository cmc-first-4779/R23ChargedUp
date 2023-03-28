// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// This command sync's our relative encoders to our absolute encoders
public class SyncEncoders extends InstantCommand {
  ShoulderSubsystem shoulder;
  WristSubsystem wrist;
  public SyncEncoders(ShoulderSubsystem shoulder, WristSubsystem wrist) {
    this.shoulder = shoulder;
    this.wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Sync our Encoders
    wrist.syncEncoders();
    shoulder.syncEncoders();
  }
}
