// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.GetRobotPosition;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.LimeLight.GetLocationOfAprilTag;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.StaticConstants.BlingConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.balanceTest;
import frc.robot.commands.sturdyBaseCommand;
import frc.robot.commands.BlingCommands.BlingSetPattern;
import frc.robot.commands.ExtenderCommands.ExtendExtender;
import frc.robot.commands.ExtenderCommands.RetractExtender;
import frc.robot.commands.ShoulderCommands.ShoulderLower;
import frc.robot.commands.ShoulderCommands.ShoulderMoveWithJoystick;
import frc.robot.commands.ShoulderCommands.ShoulderRaise;
import frc.robot.commands.ShoulderCommands.ShoulderSetPosition;
import frc.robot.commands.ShoulderCommands.ShoulderStopCommand;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commandGroups.SafeRectractToStowSCG;
import frc.robot.commandGroups.SafeSetToPositionSCG;
import frc.robot.commandGroups.StopAllPCG;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommands.IntakeEjectCommand;
import frc.robot.commands.IntakeCommands.IntakePickupCommand;
import frc.robot.commands.ShoulderCommands.ShoulderLower;
import frc.robot.commands.ShoulderCommands.ShoulderRaise;
import frc.robot.commands.ShoulderCommands.ShoulderStopCommand;
import frc.robot.commands.WristCommands.WristLower;
import frc.robot.commands.WristCommands.WristRaise;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

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
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem driveTrain = new DrivetrainSubsystem();
  SwerveModule m_frontLeftModule;
  SwerveModule m_frontRightModule;
  SwerveModule m_backLeftModule;
  SwerveModule m_backRightModule;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private boolean usePathPlanner = true;
  private boolean debugSwerve = false;
  SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  SwerveAutoBuilder autoBuilder;
  List<PathPlannerTrajectory> selectedAuto;
  SendableChooser<String> AllianceChooser = new SendableChooser<>();
  // String selectedAuto;
  // Trajectory chosenTrajectory;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ExtenderSubsystem extender = new ExtenderSubsystem(this);
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final WristSubsystem wrist = new WristSubsystem(this);
  private final BlingSubsystem bling = new BlingSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller driverStick =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller operStick =
      new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);
  
 


  HashMap<String, Command> pathPlannerEventMap = new HashMap<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    driveTrain.setDefaultCommand(new DefaultDriveCommand(
        driveTrain,
        () -> -modifyAxis(driverStick.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverStick.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverStick.getRightX())
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putBoolean("Use PathPlanner", true);
    SmartDashboard.putBoolean("Debug Swerve", false);
    // SmartDashboard.putString("selectedAuto", selectedAuto);
    
    generatePathPlannerPathGroups();
    createAutoBuilder();

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
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));
    // driverStick.L2().whileTrue(new IntakePickupCommand(intake));
    // driverStick.R2().whileTrue(new IntakeEjectCommand(intake));
    // driverStick.cross().whileTrue(new ShoulderLower(shoulder));
    // driverStick.triangle().whileTrue(new ShoulderRaise(shoulder));
    // driverStick.touchpad().onTrue(new ShoulderStopCommand(shoulder));
    // driverStick.circle().whileTrue(new ExtendExtender(extender));
    // driverStick.square().whileTrue(new RetractExtender(extender));
    // driverStick.L1().whileTrue(new WristRaise(wrist));
    // driverStick.R1().whileTrue(new WristLower(wrist));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
 
    driverStick.povUp().onTrue(new BlingSetPattern(bling, BlingConstants.BLING_PARTY_PALETTE));
    driverStick.povLeft().onTrue(new BlingSetPattern(bling, BlingConstants.BLING_VIOLET));
    driverStick.povRight().onTrue(new BlingSetPattern(bling, BlingConstants.BLING_YELLOW));
    driverStick.L3().whileTrue(new sturdyBaseCommand(driveTrain));
    driverStick.options().whileTrue(new RunCommand(driveTrain::zeroGyroscope, driveTrain));
    // m_driverController.circle().whileTrue(new balanceTest(m_drivetrainSubsystem));
    operStick.L1().whileTrue(new IntakeEjectCommand(intake));
    operStick.R1().whileTrue(new IntakePickupCommand(intake));
    operStick.povDown().onTrue(new StopAllPCG(shoulder, extender, intake));
    operStick.share().onTrue(new SafeSetToPositionSCG("HIGH_CUBE", shoulder, extender, wrist));
    operStick.options().onTrue(new SafeSetToPositionSCG("HIGH_CONE", shoulder, extender, wrist));
    operStick.square().onTrue(new SafeSetToPositionSCG("MID_CUBE", shoulder, extender, wrist));
    operStick.triangle().onTrue(new SafeSetToPositionSCG("MID_CONE", shoulder, extender, wrist));
    operStick.cross().onTrue(new SafeSetToPositionSCG("LOW_CUBE", shoulder, extender, wrist));
    operStick.circle().onTrue(new SafeSetToPositionSCG("LOW_CONE", shoulder, extender, wrist));
    operStick.L3().onTrue(new SafeSetToPositionSCG("PICKUP_CONE", shoulder, extender, wrist));
    operStick.R3().onTrue(new SafeSetToPositionSCG("PICKUP_CUBE", shoulder, extender, wrist));
    operStick.touchpad().onTrue(new SafeRectractToStowSCG(shoulder, extender, wrist) ); // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    
    //
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
    usePathPlanner = SmartDashboard.getBoolean("Use PathPlanner", false);
    if (usePathPlanner) {
      return generateAutoWithPathPlanner();
    } else {
      return manuallyGenerateAuto();
    }

  }

  /**
   * Generates the Path Planner Groups and add them to the AutoChooser
   */
  private void generatePathPlannerPathGroups() {
    List<PathPlannerTrajectory> testSquareTraj = PathPlanner.loadPathGroup("SquareTest", new PathConstraints(2, 2));
    List<PathPlannerTrajectory> testPathTraj = PathPlanner.loadPathGroup("TestPath", new PathConstraints(2, 3));
    List<PathPlannerTrajectory> Blue_Drop_Cone_And_Pickuptraj = PathPlanner.loadPathGroup("Blue Drop Cone and Pickup",
        new PathConstraints(2, 3));

    autoChooser.setDefaultOption("Test Square", testSquareTraj);
    autoChooser.addOption("TestPath", testPathTraj);
    autoChooser.addOption("Blue_Drop_Cone_And_Pickup", Blue_Drop_Cone_And_Pickuptraj);
  }

  /**
   * Creates the AutoBuilder
   */
  private void createAutoBuilder() {

    pathPlannerEventMap = new HashMap<>();
    pathPlannerEventMap.put("marker1", new PrintCommand("Passed marker 1"));
    pathPlannerEventMap.put("intakeDown", new PrintCommand("Event 2"));
    autoBuilder = new SwerveAutoBuilder(

        driveTrain::getPose, // Pose2d supplier
        driveTrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        driveTrain.geKinematics(), // SwerveDriveKinematics
        new PIDConstants(Constants.kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to
                                                              // create the X and Y PID controllers)
        new PIDConstants(Constants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to
                                                                 // create the rotation controller)
        driveTrain::drive, // Module states consumer used to output to the drive subsystem
        pathPlannerEventMap,
        true, // Should the path be automatically mirrored depending on alliance color.
              // Optional, defaults to true
        driveTrain // The drive subsystem. Used to properly set the requirements of path following
                              // commands
    );
  }

  private Command generateAutoWithPathPlanner() {
    return autoBuilder.fullAuto(autoChooser.getSelected());
  }

  /**
   * Creates a command that follows a manually generated path
   * 
   * @return
   */
  private Command manuallyGenerateAuto() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND_TRAJECTORY,
        DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveTrain.geKinematics());

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 0),
            new Translation2d(0, -1)),
        // new Translation2d(2, Rotation2d.fromDegrees(-90))),
        new Pose2d(1, -1, Rotation2d.fromDegrees(90)),

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
        driveTrain::getPose,
        driveTrain.geKinematics(),
        xController,
        yController,
        thetaController,
        driveTrain::drive,
        driveTrain);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> driveTrain.stopModules()));

  }

  /**
   * Checks to make sure a setting is outside of our dedband range
   * 
   * @param value    the value to check
   * @param deadband the range of our deadband
   * @return 0 if inside the deadband, the value if outside the deadband
   *         normalized
   */
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

  /**
   * Gets the drive train subsystem
   */
  public DrivetrainSubsystem getDriveTrainSubsystem() {
    return driveTrain;
  }

  /**
   * Gets the drive train subsystem
   */


 /**
   * Gets the current postion of the shoulder
   * 
   * @return
   */
  public double getShoulderPosition() {
    return shoulder.getShoulderPosition();
  }

 

    /**
   * Gets the current postion of the extender
   * 
   * @return
   */
  public double getExtenderPosition() {
    return extender.getExtenderPosition();
  }

    /**
   * Gets the current postion of the wrist
   * 
   * @return
   */
  public double getWristPosition() {
    return wrist.getWristPosition();
  }


  

 

  public LimelightSubsystem getLimelightSubsystem() {
    return limelight;
  }

  public DriverStation.Alliance getAlliance(){ 
    return DriverStation.getAlliance();

}

  // public String getAllianceChooseString(){
  //   return allianceChooser.getSelected();
  // }
  
  public void setButtons() {
    if (getAlliance() == Alliance.Red) {
      driverStick.triangle().onTrue(new GetLocationOfAprilTag(limelight, 1));
      driverStick.circle().onTrue(new GetLocationOfAprilTag(limelight, 2));
      driverStick.cross().onTrue(new GetLocationOfAprilTag(limelight, 3));
      driverStick.square().onTrue(new GetLocationOfAprilTag(limelight, 4));
      }
      else {
      driverStick.triangle().onTrue(new GetLocationOfAprilTag(limelight, 5));
      driverStick.circle().onTrue(new GetLocationOfAprilTag(limelight, 6));
      driverStick.cross().onTrue(new GetLocationOfAprilTag(limelight, 7));
      driverStick.square().onTrue(new GetLocationOfAprilTag(limelight, 0));
      }
  }
}

