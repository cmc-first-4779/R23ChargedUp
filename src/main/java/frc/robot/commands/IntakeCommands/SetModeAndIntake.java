// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetModeAndIntake extends CommandBase {
  private IntakeSubsystem intake;
  private boolean cubeMode;
  /** Creates a new SetModeAndIntake. */
  public SetModeAndIntake(IntakeSubsystem intake, boolean cubeMode) {
    this.intake = intake;
    this.cubeMode = cubeMode;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.setCubeMode(cubeMode);
    if (cubeMode) {
      intake
    } else {
      Robot.setCubeMode(fals)
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
