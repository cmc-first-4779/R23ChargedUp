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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.StaticConstants.BlingConstants;
import frc.robot.commands.BlingCommands.BlingSetPattern;
import frc.robot.commands.DriveTrainCommands.DefaultDriveCommand;
import frc.robot.commands.DriveTrainCommands.ResetGyro;
import frc.robot.commands.DriveTrainCommands.AutoBalanceFaster;
import frc.robot.commands.DriveTrainCommands.sturdyBaseCommand;
import frc.robot.commands.ExtenderCommands.ExtendExtender;
import frc.robot.commands.ExtenderCommands.RetractExtender;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commandGroups.SafeRectractToStowSCG;
import frc.robot.commandGroups.SafeSetToPositionSCG;
import frc.robot.commandGroups.SetToPositionPCG;
import frc.robot.commandGroups.StopAllPCG;
import frc.robot.commands.IntakeCommands.AutoIntakeSetSpeed;
import frc.robot.commands.IntakeCommands.IntakeAutoSense;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import frc.robot.commands.LimelightCommands.LimelightInitForDriver;
import frc.robot.commands.LimelightCommands.LimelightTargetDeploy;
import frc.robot.commands.MiscCommands.SyncEncoders;
import frc.robot.commands.ShoulderCommands.ShoulderLower;
import frc.robot.commands.ShoulderCommands.ShoulderRaise;
import frc.robot.commands.WristCommands.WristLower;
import frc.robot.commands.WristCommands.WristRaise;
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

  // Set to use Path Planner
  private boolean usePathPlanner = true;
  // private boolean debugSwerve = false;
  SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  SwerveAutoBuilder autoBuilder;
  List<PathPlannerTrajectory> selectedAuto;
  SendableChooser<String> AllianceChooser = new SendableChooser<>();

  // Declare our other subsystems
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ExtenderSubsystem extender = new ExtenderSubsystem(this);
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final WristSubsystem wrist = new WristSubsystem(this);
  private final BlingSubsystem bling = new BlingSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  // Declare our two PS4 Controllers
  // DriverStick on USB Port 0
  // OperStick on USB Port 1
  private final CommandPS4Controller driverStick = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller operStick = new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  // Init the HashMap for Path Planner Commands
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
        () -> -modifyAxis(driverStick.getLeftY() * Constants.DRIVETRAIN_THROTTLE)
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverStick.getLeftX() * Constants.DRIVETRAIN_THROTTLE)
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverStick.getRightX() * Constants.DRIVETRAIN_THROTTLE)
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Default Command for the Intake is: STOP
    intake.setDefaultCommand(new IntakeStopCommand(intake));
    // Turning the LimeLight Off for Now.
    //limelight.initLimelightforDriver();
    //  Set our Bling Default Pattern
    bling.setDefaultCommand(new BlingSetPattern(bling, BlingConstants.BLING_GLITTER_PALETTE));
    // Configure the trigger bindings
    configureBindings();

    // Send our Auto Chooser options to the Dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putBoolean("Use PathPlanner", true);
    SmartDashboard.putBoolean("Debug Swerve", false);
    // SmartDashboard.putString("selectedAuto", selectedAuto);

    // Generate Path Planner Groups
    generatePathPlannerPathGroups();
    createAutoBuilder();

    shoulder.syncEncoders();
    wrist.syncEncoders();

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
    // DriveStick Buttons
    driverStick.L1().whileTrue(new WristRaise(wrist));
    driverStick.R1().whileTrue(new WristLower(wrist));
    driverStick.L2().whileTrue(new ShoulderRaise(shoulder));
    driverStick.R2().whileTrue(new ShoulderLower(shoulder));
    driverStick.L3().whileTrue(new ExtendExtender(extender));
    driverStick.R3().whileTrue(new RetractExtender(extender));
    driverStick.options().whileTrue(new sturdyBaseCommand(driveTrain));
    //driverStick.share().onTrue(new SyncEncoders(shoulder, wrist));
    driverStick.povDown().whileTrue(new RunCommand(driveTrain::zeroGyroscope, driveTrain));
    driverStick.povRight().onTrue(new BlingSetPattern(bling, BlingConstants.BLING_VIOLET));
    driverStick.povLeft().onTrue(new BlingSetPattern(bling, BlingConstants.BLING_YELLOW));
    driverStick.povUp().onTrue(new BlingSetPattern(bling, BlingConstants.BLING_PARTY_PALETTE));
    driverStick.cross().whileTrue(new LimelightTargetDeploy(driveTrain, limelight, "DOUBLE_HPS_RIGHT"));
    driverStick.triangle().whileTrue(new LimelightTargetDeploy(driveTrain, limelight, "DOUBLE_HPS_RIGHT"));
    driverStick.square().whileTrue(new LimelightTargetDeploy(driveTrain, limelight, "CUBE"));
    driverStick.circle().whileTrue(new LimelightTargetDeploy(driveTrain, limelight, "TELEOP_CONE"));

    //driverStick.povRight().onTrue(new LimelightTargetDeploy(driveTrain, limelight, "CUBE"));
    driverStick.touchpad().whileTrue(new AutoBalanceFaster(driveTrain));


    // OperStick Buttons
    // operStick.L1().whileTrue(new IntakeSetSpeed(intake, "INTAKE_CUBE"));
    // operStick.R1().whileTrue(new IntakeSetSpeed(intake, "INTAKE_CONE"));
    operStick.L1().whileTrue(new IntakeAutoSense(intake, "CUBE"));
    operStick.R1().whileTrue(new IntakeAutoSense(intake, "CONE"));
    operStick.povDown().onTrue(new StopAllPCG(shoulder, extender, wrist, intake));
    operStick.share().onTrue(new SetToPositionPCG("HIGH_CUBE", shoulder, extender, wrist));
    operStick.options().onTrue(new SetToPositionPCG("HIGH_CONE", shoulder, extender, wrist));
    operStick.square().onTrue(new SetToPositionPCG("MID_CUBE", shoulder, extender, wrist));
    operStick.triangle().onTrue(new SetToPositionPCG("MID_CONE", shoulder, extender, wrist));
    operStick.cross().onTrue(new SafeSetToPositionSCG("LOW_CUBE", shoulder, extender, wrist));
    operStick.circle().onTrue(new SafeSetToPositionSCG("LOW_CONE", shoulder, extender, wrist));
    operStick.R3().onTrue(new SafeSetToPositionSCG("PICKUP_CONE", shoulder, extender, wrist));
    operStick.L3().onTrue(new SafeSetToPositionSCG("PICKUP_CUBE", shoulder, extender, wrist));
    operStick.L2().onTrue(new SafeSetToPositionSCG("HUMAN_PLAYER_STATION", shoulder, extender, wrist));
    operStick.touchpad().onTrue(new SafeRectractToStowSCG(shoulder, extender, wrist)); 
    operStick.povUp().onTrue(new SetToPositionPCG("DOUBLE_HPS", shoulder, extender, wrist));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    usePathPlanner = SmartDashboard.getBoolean("Use PathPlanner", true);
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
    List<PathPlannerTrajectory> Station_2_Engage = PathPlanner.loadPathGroup("Station 2 Engage",
        new PathConstraints(2, 2));
    List<PathPlannerTrajectory> Station_1_Cube_and_Pickuptraj = PathPlanner.loadPathGroup("Station 1 Cube and Pickup",
        new PathConstraints(2, 3));
    List<PathPlannerTrajectory> Station_1_Cone_And_Pickuptraj = PathPlanner.loadPathGroup(
        "Station 1 Cone and Pickup",
        new PathConstraints(2, 3));
    List<PathPlannerTrajectory> Station_1_Cube_and_Engage = PathPlanner.loadPathGroup("Station 1 Cube and Engage",
        new PathConstraints(2, 2));
    List<PathPlannerTrajectory> Station_3_Cone_and_Pickup = PathPlanner.loadPathGroup("Station 3 Cone and Pickup",
        new PathConstraints(2, 2));

    autoChooser.setDefaultOption("Station 2 Engage", Station_2_Engage);
    autoChooser.addOption("Station 1 Cube and Pickup", Station_1_Cube_and_Pickuptraj);
    autoChooser.addOption("Station 1 Cone And Pickup", Station_1_Cone_And_Pickuptraj);
    autoChooser.addOption("Station 1 Cube and Engage", Station_1_Cube_and_Engage);
    autoChooser.addOption("Station 3 Cone and Pickup", Station_3_Cone_and_Pickup);
  }

  /**
   * Creates the AutoBuilder
   */
  private void createAutoBuilder() {

    pathPlannerEventMap = new HashMap<>();
    // pathPlannerEventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // pathPlannerEventMap.put("intakeDown", new PrintCommand("Event 2"));
    pathPlannerEventMap.put("Cone High", new SetToPositionPCG("HIGH_CONE", shoulder, extender, wrist));
    pathPlannerEventMap.put("Cube High", new SetToPositionPCG("AUTO_HIGH_CUBE", shoulder, extender, wrist));
    pathPlannerEventMap.put("Cone Mid", new SetToPositionPCG("MID_CONE", shoulder, extender, wrist));
    pathPlannerEventMap.put("Cube Mid", new SetToPositionPCG("MID_CUBE", shoulder, extender, wrist));
    pathPlannerEventMap.put("Cone Low", new SafeSetToPositionSCG("LOW_CONE", shoulder, extender, wrist));
    pathPlannerEventMap.put("Cube Low", new SafeSetToPositionSCG("LOW_CUBE", shoulder, extender, wrist));
    // pathPlannerEventMap.put("Cone Pickup", new
    // AutoSafeSetToPositionSCG("PICKUP_CONE", shoulder, extender, wrist));
    // pathPlannerEventMap.put("Cube Pickup", new
    // AutoSafeSetToPositionSCG("PICKUP_CUBE", shoulder, extender, wrist));
    pathPlannerEventMap.put("Cone Pickup", new SafeSetToPositionSCG("PICKUP_CONE", shoulder, extender, wrist));
    pathPlannerEventMap.put("Cube Pickup", new SafeSetToPositionSCG("AUTO_PICKUP_CUBE", shoulder, extender, wrist));
    pathPlannerEventMap.put("Safe Retract", new SafeRectractToStowSCG(shoulder, extender, wrist));
    pathPlannerEventMap.put("Eject Cone", new AutoIntakeSetSpeed(intake, "EJECT_CONE"));
    pathPlannerEventMap.put("Eject Cube", new AutoIntakeSetSpeed(intake, "EJECT_CUBE"));
    pathPlannerEventMap.put("Pickup Cone", new AutoIntakeSetSpeed(intake, "INTAKE_CONE"));
    pathPlannerEventMap.put("Pickup Cube", new AutoIntakeSetSpeed(intake, "INTAKE_CUBE"));
    pathPlannerEventMap.put("Wait Short", new WaitCommand(0.45));
    pathPlannerEventMap.put("Wait Long", new WaitCommand(1));
    pathPlannerEventMap.put("Auto Balance", new AutoBalanceFaster(driveTrain));
    pathPlannerEventMap.put("Reset Gyro", new ResetGyro(driveTrain));
    pathPlannerEventMap.put("Sturdy Base", new sturdyBaseCommand(driveTrain));
    pathPlannerEventMap.put("LL Cube Deploy", new LimelightTargetDeploy(driveTrain, limelight, "CUBE"));
    pathPlannerEventMap.put("LL Cone Deploy", new LimelightTargetDeploy(driveTrain, limelight, "AUTON_CONE"));
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

  // Returns the alliance color that we are on
  public DriverStation.Alliance getAlliance() {
    return DriverStation.getAlliance();
  }



  // Returns the DriveTrain Subsystem to other classes
  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return driveTrain;
  }

  // Returns the Shoulder Subsystem to other classes
  public ShoulderSubsystem getShoulderSubsystem() {
    return shoulder;
  }

  // Returns the Extender Subsystem to other classes
  public ExtenderSubsystem getExtenderSubsystem() {
    return extender;
  }

  // Returns the Wrist Subsystem to other classes
  public WristSubsystem getWristSubsystem() {
    return wrist;
  }

  // Returns the Limelight Subsystem to other classes
  public LimelightSubsystem getLimelightSubsystem() {
    return limelight;
  }

  // Returns the Bling Subsystem to other classes
  public BlingSubsystem getBlingSubsystem() {
    return bling;
  }


}
