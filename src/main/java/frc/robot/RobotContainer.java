// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commandGroups.SafeSetToPositionSCG;
import frc.robot.commandGroups.StopAllPCG;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommands.IntakeEjectCommand;
import frc.robot.commands.IntakeCommands.IntakePickupCommand;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.ExtenderCommands.ExtendByJoystick;
import frc.robot.commands.ExtenderCommands.ExtendExtender;
import frc.robot.commands.ExtenderCommands.ExtenderSetPosition;
import frc.robot.commands.ExtenderCommands.ExtenderStopCommand;
import frc.robot.commands.ExtenderCommands.RetractExtender;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.commands.ShoulderCommands.ShoulderLower;
import frc.robot.commands.ShoulderCommands.ShoulderMoveWithJoystick;
import frc.robot.commands.ShoulderCommands.ShoulderRaise;
import frc.robot.commands.ShoulderCommands.ShoulderSetPosition;
import frc.robot.commands.ShoulderCommands.ShoulderStopCommand;
import frc.robot.commands.ShoulderCommands.ShoulderTESTCommand;
import frc.robot.subsystems.ShoulderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ExtenderSubsystem extender = new ExtenderSubsystem(this);
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    m_driverController.L1().whileTrue(new IntakeEjectCommand(intake));
    m_driverController.R1().whileTrue(new IntakePickupCommand(intake));
    m_driverController.povDown().onTrue(new StopAllPCG(shoulder, extender, intake));
    m_driverController.share().onTrue(new SafeSetToPositionSCG("HIGH_CUBE", shoulder, extender));
    m_driverController.options().onTrue(new SafeSetToPositionSCG("HIGH_CONE", shoulder, extender));
    m_driverController.square().onTrue(new SafeSetToPositionSCG("MID_CUBE", shoulder, extender));
    m_driverController.triangle().onTrue(new SafeSetToPositionSCG("MID_CONE", shoulder, extender));
    m_driverController.cross().onTrue(new SafeSetToPositionSCG("LOW_CUBE", shoulder, extender));
    m_driverController.circle().onTrue(new SafeSetToPositionSCG("LOW_CONE", shoulder, extender));
    m_driverController.touchpad().onTrue(new SafeSetToPositionSCG("STOW", shoulder, extender));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  /**
   * Gets the current postion of the arm
   * 
   * @return
   */
  public double getShoulderPosition() {
    return shoulder.getShoulderPos();
  }

}
