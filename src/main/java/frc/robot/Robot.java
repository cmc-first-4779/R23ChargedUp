// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.reflection.ClassInformationSender;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  SwerveControllerCommand autoCommand = null;

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> sideChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Get the names of all the autos and then add them to a chooser
    AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

    // Ensure the second String is the name of the folder where your sided autos are
    // located
    sideChooser.setDefaultOption("Blue", "blue");
    sideChooser.addOption("Red", "red");

    System.out.println("Should be putting autochooser on SmartDashboard");
    SmartDashboard.putData("Auto choices", autoChooser);
    SmartDashboard.putData("Red or Blue", sideChooser);

    AutonomousContainer.getInstance().setDebugPrints(true);
    AutonomousContainer.getInstance().initialize(
        true,
        new CommandTranslator(
            (trajectory) -> {
              autoCommand = createSwerveControllerCommand(trajectory);
              CommandScheduler.getInstance().schedule(autoCommand);
            },
            m_robotContainer.getDriveSubsystem()::stopModules,
            null,
            () -> !CommandScheduler.getInstance().isScheduled(autoCommand), // The command is finished when it is no
                                                                            // longer scheduled
            null,
            m_robotContainer.getDriveSubsystem()::resetOdometry,
            true),
        false,
        this,
        m_robotContainer);

            // Get the names of all the autos and then add them to a chooser
    AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

    // Ensure the second String is the name of the folder where your sided autos are
    // located
    sideChooser.setDefaultOption("Blue", "blue");
    sideChooser.addOption("Red", "red");

    System.out.println("Should be putting autochooser on SmartDashboard");
    SmartDashboard.putData("Auto choices", autoChooser);
    SmartDashboard.putData("Red or Blue", sideChooser);

  }

  public SwerveControllerCommand createSwerveControllerCommand(Trajectory trajectory) {
    DrivetrainSubsystem drive = m_robotContainer.getDriveSubsystem();
    return new SwerveControllerCommand(
        trajectory,
        () -> (getPose()),
        getKinematics(),
        new PIDController(Constants.kPXController, 0, 0),
        new PIDController(Constants.kPYController, 0, 0),
        new ProfiledPIDController(Constants.kPThetaController, 0, 0, new TrapezoidProfile.Constraints(1, 1)),
        AutonomousContainer.getCommandTranslator()::getWantedRotation,
        drive::drive,
        drive // Make sure you add Drive as a requirement so that the controller doesn't
              // try to control the modules while
    // the robot is running an autonomous command
    );
  }

  private SwerveDriveKinematics getKinematics() {
    return Constants.kDriveKinematics;
  }

  private Pose2d getPose() {
    return m_robotContainer.getDriveSubsystem().getPose();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

    //Run the autos
    String autoName = autoChooser.getSelected();
    if (autoName == null) {
        autoName = "Kevin"; // Default auto if none is selected
    }
    // If it can't find a sided auto it will try to find a non-sided auto
    AutonomousContainer.getInstance().runAutonomous(autoName, sideChooser.getSelected(), true); // The last boolean is about allowing network autos to run, keep this set to true unless you have a reason to disable them.

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    ClassInformationSender.updateReflectionInformation("frc");
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
