// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BlingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StaticConstants.BlingConstants;
import frc.robot.subsystems.BlingSubsystem;

  //  Toggles our Bling Lights from Yellow to Violet
  //  Using this to save buttons on our controller

public class ToggleBling extends CommandBase {
  BlingSubsystem bling;
  double currentBling;
  /** Creates a new ToggleBling. */
  public ToggleBling(BlingSubsystem bling) {
    this.bling = bling;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(bling);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Get the current value of our BLING LEDs
    currentBling = bling.getBlingPattern();
    //  If our current bling is yellow...
    if (currentBling == BlingConstants.BLING_YELLOW) {
      // then set them to violet / purple
      bling.setBlingPattern(BlingConstants.BLING_VIOLET);
    } else {  // Our bling must be already violet / purple
      //  So set them to Yellow.
      bling.setBlingPattern(BlingConstants.BLING_YELLOW);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
