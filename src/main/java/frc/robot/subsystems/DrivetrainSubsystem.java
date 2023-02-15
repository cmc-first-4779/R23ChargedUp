// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

// import javax.annotation.Nonnull;
// import javax.annotation.Nullable;

import com.ctre.phoenix.sensors.Pigeon2;
import com.dacubeking.AutoBuilder.robot.annotations.AutoBuilderAccessible;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    // DONE Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
    public static final double MAX_VELOCITY_METERS_PER_SECOND_TRAJECTORY = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

    // Max acceleration needed for TrajectoryConfig constructor. Using value from 0
    // to Autonomous example
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0; // original line of code
    // public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.5;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    // By default we use a Pigeon for our gyroscope. But if you use another
    // gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating
    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    // Don't Remove following if you are using a Pigeon
    // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
    Pigeon2 gyroscope = new Pigeon2(DRIVETRAIN_PIGEON_ID);
    Double m_pigeonPitch;
    Double m_pigeonYaw;
    Double m_pigeonRoll;
    // Uncomment following if you are using a NavX
    // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX
    // connected over MXP

    private final SwerveDriveOdometry odometry;

    // Field object to keep track of location of bot.
    private final Field2d field = new Field2d();

    // These are our modules. We initialize them in the constructor.
    public SwerveModule frontLeftModule;
    public SwerveModule frontRightModule;
    public SwerveModule backLeftModule;
    public SwerveModule backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SwerveModuleState[] states;

    public DrivetrainSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        SmartDashboard.putData("Field", field);
        System.out.println("Running First Module");
        frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Left Module:", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
                .build();
        System.out.println("Running Second Module");
        frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Right Module:", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
                .build();
        System.out.println("Running Third Module");
        backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Left Module:", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
                .build();
        System.out.println("Running Fourth Module");
        backRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Right Module:", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
                .build();

        // odometery = new SwerveDriveOdometry(Constants.kDriveKinematics,
        // new Rotation2d(0), new SwereModulePosition[] {
        // m_frontLeftModule.getPosition(),

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(gyroscope.getYaw()),
                new SwerveModulePosition[] { frontLeftModule.getPosition(), frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition() });

    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
    // // Inline construction of command goes here.
    // // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//     () -> {
//     gyroscope.setYaw(0.0);
//     });
    // Don't Remove following if you are using a Pigeon
//     m_pigeon.setFusedHeading(0.0);
    gyroscope.setYaw(0.0);

    // // Uncomment Following if you are using a NavX
    // // m_navx.zeroYaw();
    }

//     public void zeroGyroscope() {
//         odometry.resetPosition(
//                 Rotation2d.fromDegrees(gyroscope.getYaw()),
//                 new SwerveModulePosition[] { frontLeftModule.getPosition(), frontRightModule.getPosition(),
//                         backLeftModule.getPosition(), backRightModule.getPosition() },
//                 new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)));
//     }

    public Rotation2d getGyroscopeRotation() {
        // Don't Remove Follwoing if you are using a Pigeon
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
        return Rotation2d.fromDegrees(gyroscope.getYaw());

        // Uncomment following if you are using a NavX
        // if (m_navx.isMagnetometerCalibrated()) {
        // // We will only get valid fused headings if the magnetometer is calibrated
        // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        // }
        //
        // // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public Rotation2d getGyroscopePitch() {
        return Rotation2d.fromDegrees(gyroscope.getPitch());
    }

    public Rotation2d getGyroscopeRoll() {
        return Rotation2d.fromDegrees(gyroscope.getRoll());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
        states = kinematics.toSwerveModuleStates(m_chassisSpeeds);
    }

    /**
     * Drive method that can take in SwerveModuleState array. Needed for auton since
     * we have to provide a consumer of SwerveModuleState[]
     * 
     * @param states the states of the swerve modules
     */
    public void drive(SwerveModuleState[] states) {
        this.states = states;
    }

    @Override
    public void periodic() {
        m_pigeonPitch = gyroscope.getPitch();
        m_pigeonYaw = gyroscope.getYaw();
        m_pigeonRoll = gyroscope.getRoll();
        SmartDashboard.putNumber("Robot Pitch", m_pigeonPitch);
        SmartDashboard.putNumber("Robot Yaw", m_pigeonYaw);
        SmartDashboard.putNumber("Robot Roll", m_pigeonRoll);
        SmartDashboard.updateValues();
        // Moving this line into the drive() method
        // SwerveModuleState[] states =
        // m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        // Trying update robot position on field image on dashboard
        field.setRobotPose(odometry.getPoseMeters());

        if (states == null) {
            return;
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    /**
     * Gets the current pose of the robot
     * 
     * @return
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    @AutoBuilderAccessible
    // I'm 100% sure I have this method doing what we want. Need to test it.
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose.getRotation(),
                new SwerveModulePosition[] { frontLeftModule.getPosition(), frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition() },
                pose);
    }
    @AutoBuilderAccessible
    public void stopModules() {
        frontLeftModule.set(0, frontLeftModule.getSteerAngle());
        frontRightModule.set(0, frontRightModule.getSteerAngle());
        backLeftModule.set(0, backLeftModule.getSteerAngle());
        backRightModule.set(0, backRightModule.getSteerAngle());
    }

    public double getHeading() {
        return Math.IEEEremainder(gyroscope.getYaw(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    @AutoBuilderAccessible
    public void setAutoRotation(Rotation2d rotation){
        currentAutoTrajectoryLock.lock();
        try {
            autoTargetHeading = rotation;
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
        System.out.println("new rotation" + rotation.getDegrees());
    }
    @AutoBuilderAccessible
    synchronized public boolean isFinished() {
        return driveState == DriveState.STOP || driveState == DriveState.DONE || driveState == DriveState.TELEOP;
    }
    @AutoBuilderAccessible
    public double getAutoElapsedTime() {
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    public double getPitch() {
        return gyroscope.getPitch();
    }

    public double getRoll() {
        return gyroscope.getRoll();
    }

    public void driveStraightSlow() {
        frontLeftModule.set(2, 0);
        frontRightModule.set(2, 0);
        ;
        backLeftModule.set(2, 0);
        ;
        backRightModule.set(2, 0);
    }

    public void driveBackwardsSlow() {
        frontLeftModule.set(-2, 0);
        frontRightModule.set(-2, 0);
        ;
        backLeftModule.set(-2, 0);
        ;
        backRightModule.set(-2, 0);
    }

    public void sturdyBase() {
        backRightModule.set(0, 45);
        ;
        backLeftModule.set(0, -45);
        ;
        frontRightModule.set(0, -45);
        ;
        frontLeftModule.set(0, 45);
        ;
    }


    public enum DriveState {
        TELEOP, TURN, HOLD, DONE, RAMSETE, STOP
    }
    public /*@Nonnull*/ DriveState driveState;

    public synchronized void setDriveState(/*@Nonnull*/ DriveState driveState) {
        this.driveState = driveState;
    }


    double autoStartTime;

    private final ReentrantLock swerveAutoControllerLock = new ReentrantLock();
    private /*@Nullable*/ HolonomicDriveController swerveAutoController;
    boolean swerveAutoControllerInitialized = false;
    @AutoBuilderAccessible
    public void setAutoPath(Trajectory trajectory) {
        currentAutoTrajectoryLock.lock();
        try {
            swerveAutoControllerInitialized = false;
            setDriveState(DriveState.RAMSETE);
            this.currentAutoTrajectory = trajectory;
            this.isAutoAiming = false;
            autoStartTime = Timer.getFPGATimestamp();
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
    }

    Trajectory currentAutoTrajectory;
    final Lock currentAutoTrajectoryLock = new ReentrantLock();
    volatile Rotation2d autoTargetHeading;
    private volatile boolean isAutoAiming = false;

    private static final /*@NotNull*/ DrivetrainSubsystem INSTANCE = new DrivetrainSubsystem();
    public static /*@NotNull*/ DrivetrainSubsystem getInstance() {
        return INSTANCE;
    }
}
