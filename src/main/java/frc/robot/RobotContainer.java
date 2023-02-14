// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.Util;
import com.dacubeking.AutoBuilder.robot.annotations.AutoBuilderAccessible;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.balanceTest;
import frc.robot.commands.sturdyBaseCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  SwerveModule m_frontLeftModule;
  SwerveModule m_frontRightModule;
  SwerveModule m_backLeftModule;
  SwerveModule m_backRightModule;

  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

 // @AutoBuilderAccessible this method enables the command to be used in AutoBuilder
  @AutoBuilderAccessible
  private final balanceTest balancetest = new balanceTest(m_drivetrainSubsystem);

  @AutoBuilderAccessible
  private final DriveStraightCommand driveStraight = new DriveStraightCommand(m_drivetrainSubsystem);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driverController.b().whileTrue(new sturdyBaseCommand(m_drivetrainSubsystem, m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule));
    // // Back button zeros the gyroscope
    // m_driverController.back().onTrue(m_drivetrainSubsystem.zeroGyroscope());
    // m_driverController.x().onTrue(new balanceTest(m_drivetrainSubsystem)); // This button X on the controller
    new Trigger(m_driverController::getBButton).whileTrue(new sturdyBaseCommand(m_drivetrainSubsystem, m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule));
    // Back button zeros the gyroscope
    new Trigger(m_driverController::getXButton).onTrue(new RunCommand(m_drivetrainSubsystem::zeroGyroscope));
    new Trigger(m_driverController::getAButton).whileTrue(new balanceTest(m_drivetrainSubsystem)); // This button A on the controller
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND_TRAJECTORY,
                DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        .setKinematics(Constants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, -1),
                        new Translation2d(2, 0),
                        new Translation2d(3, -1)),
                        // new Translation2d(2.5, Rotation2d.fromDegrees(-90))),
                new Pose2d(4, 0, Rotation2d.fromDegrees(360)),

                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(Constants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_drivetrainSubsystem::getPose,
                Constants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_drivetrainSubsystem::drive,
                m_drivetrainSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));
  
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
