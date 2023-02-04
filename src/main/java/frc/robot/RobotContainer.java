// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.StaticConstants.BlingConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.StaticConstants.HardwareMapConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.BlingCommands.BlingSetPatternCommand;
import frc.robot.commands.WristCommands.DownWristCommand;
import frc.robot.commands.WristCommands.StopWristCommand;
import frc.robot.commands.WristCommands.UpWristCommand;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final WristSubsystem m_Wristsubsystem = new WristSubsystem();
  private final BlingSubsystem m_BlingSubsystem = new BlingSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_operstick = new CommandPS4Controller(
      OperatorConstants.kDriverControllerPort);

  private final StopWristCommand stopWristCommand = new StopWristCommand(m_Wristsubsystem);
  private final UpWristCommand upWrist = new UpWristCommand(m_Wristsubsystem);
  private final DownWristCommand downWrist = new DownWristCommand(m_Wristsubsystem);
  private final BlingSetPatternCommand blingSetPatternViolet = new BlingSetPatternCommand(m_BlingSubsystem, BlingConstants.BLING_VIOLET);
  private final BlingSetPatternCommand blingSetPatternYellow = new BlingSetPatternCommand(m_BlingSubsystem, BlingConstants.BLING_YELLOW);
  private final BlingSetPatternCommand blingSetPatternParty = new BlingSetPatternCommand(m_BlingSubsystem, BlingConstants.BLING_PARTY_PALETTE);
  // Declare our PS4 Controllers for the Driver and Operator
  // PS4Controller m_driverStick = new
  // PS4Controller(HardwareMapConstants.DRIVERSTICK_USB_PORT);
  // PS4Controller m_operStick = new
  // PS4Controller(HardwareMapConstants.OPERSTICK_USB_PORT);

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
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // new JoystickButton(m_operStick,
    // PS4Controller.Button.kR1.value).whenPressed(stopWristCommand)(turretMotorClockwiseCommand);
    m_operstick.R1().whileTrue(upWrist);
    m_operstick.L1().whileTrue(downWrist);
    m_operstick.share().whileTrue(stopWristCommand);
    m_operstick.L3().whileTrue(blingSetPatternViolet);
    m_operstick.R3().whileTrue(blingSetPatternYellow);
    m_operstick.options().whileTrue(blingSetPatternParty);
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
}
