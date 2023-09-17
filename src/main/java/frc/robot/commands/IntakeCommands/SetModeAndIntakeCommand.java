// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class SetModeAndIntakeCommand extends CommandBase {
  private RobotContainer robotContainer;
  private IntakeSubsystem intake;
  private boolean cubeMode;
  /** Creates a new SetModeAndIntake. */
  public SetModeAndIntakeCommand(IntakeSubsystem intake, RobotContainer container, boolean cubeMode) {
    this.intake = intake;
    this.robotContainer = container;
    this.cubeMode = cubeMode;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SetModeAndIntake: Going to set cubeMode to " + cubeMode);
    robotContainer.setCubeMode(cubeMode);
    if (cubeMode) {
      intake.intakeRun(Constants.INTAKE_CUBE_SPEED);
    } else {
      intake.intakeRun(Constants.INTAKE_CONE_SPEED);
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
