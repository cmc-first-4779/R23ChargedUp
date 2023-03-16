// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static WPI_TalonSRX armMaster;
  public static WPI_TalonSRX armFollower;

  static double kDt = 0.02;

  

  public static DutyCycleEncoder armEncoder = new DutyCycleEncoder(0); 
  

  public static double maxGravityFF = 0.1;
  static double kMeasuredPosHorizontal = 11650; //Position measured when arm is horizontal
  static double kTicksPerDegree = 8192 / 360; //Sensor is 1:1 with arm rotation
  static double currentPos = armEncoder.getDistance();
  

public static  double Scalar() {
  armEncoder.setDistancePerRotation(8192);
  currentPos = armEncoder.getDistance();
  double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
  double radians = java.lang.Math.toRadians(degrees);
  double cosineScalar = java.lang.Math.cos(radians);
  
  return cosineScalar;
}


  public static TalonSRXConfiguration config2 = new TalonSRXConfiguration();

  static double position0 = 1.4;
  static double position1 = .95;
  static double position2 = -1.7;
  static double position3 = .3;
  static double position4 = .4;
  static double position5 = .5;

  //PID Constants

static double Kp = 1;
static double Ki = 0;
static double Kd = 0;

static PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
   
  static TrapezoidProfile.Constraints m_constraints =   new TrapezoidProfile.Constraints(20, 35);
public static ProfiledPIDController m_controller =   new ProfiledPIDController(Kp, Ki, Kd, m_constraints, kDt);
  

  public static void Distance() {
    armEncoder.setDistancePerRotation(8192);
    currentPos = armEncoder.getDistance();
   
    //throughBore.setDistancePerRotation(1.0 / 360.0 * 2.0 * Math.PI * .028);

    SmartDashboard.putNumber("Arm Encoder", currentPos);
    //SmartDashboard.putNumber("Arm Degrees", degrees); 
  }
  
  public Arm() {
    armMaster = new WPI_TalonSRX(1);
  }

  public static void MoveArm() {

  
    /*armMaster.config_kP(0, Kp, 30);
    armMaster.config_kI(0, Ki, 30);
    armMaster.config_kD(0, Kd, 30);
    armMaster.config_kF(0, 0, 30);
*/
armEncoder.setDistancePerRotation(1);
double currPos1Rot = armEncoder.getDistance();
double output = (/*-maxGravityFF*Scalar()) + */m_controller.calculate(currPos1Rot));


    if (RobotContainer.operator.getAButton() == true) {
    // armMaster.set(ControlMode.MotionMagic, position0, DemandType.ArbitraryFeedForward, -maxGravityFF * Scalar());
m_controller.setGoal(1);
if (currPos1Rot <= .98) {
  if (output < 0) {
    output = 0;
  }
}
armMaster.set(output);
    }
    if (RobotContainer.operator.getBButton() == true) {
      m_controller.setGoal(.95);
      armMaster.set(output);
    }
    if (RobotContainer.operator.getXButton() == true) {
      m_controller.setGoal(1.5);
      armMaster.set(output);
    }
    SmartDashboard.putNumber("Arm Output", output);
    SmartDashboard.putNumber("currPos1Rot", currPos1Rot);
   /* else if (RobotContainer.operator.getBButtonPressed() == true) {
      armMaster.set(ControlMode.MotionMagic, position1, DemandType.ArbitraryFeedForward, maxGravityFF * Scalar());
      m_controller.setGoal(position1);
    }
    else if (RobotContainer.operator.getXButtonPressed() == true) {
      armMaster.set(ControlMode.MotionMagic, position2, DemandType.ArbitraryFeedForward, maxGravityFF * Scalar());
      m_controller.setGoal(position2);
    }
    else if (RobotContainer.operator.getYButtonPressed() == true) {
      armMaster.set(ControlMode.MotionMagic, position3, DemandType.ArbitraryFeedForward, maxGravityFF * Scalar());
      m_controller.setGoal(position3);
    }
    else if (RobotContainer.operator.getRawButtonPressed(7) == true) {
      armMaster.set(ControlMode.MotionMagic, position4, DemandType.ArbitraryFeedForward, maxGravityFF * Scalar());
      m_controller.setGoal(position4);
    }
    else if (RobotContainer.operator.getRawButtonPressed(8) == true) {
      armMaster.set(ControlMode.MotionMagic, position5, DemandType.ArbitraryFeedForward, maxGravityFF * Scalar());
      m_controller.setGoal(position5);
    }
  */

  }
  public static void Voltages() {
    double current4 = pdp.getCurrent(4);
    SmartDashboard.putNumber("Manipulator", current4);
  

    double current3 = pdp.getCurrent(3);
    SmartDashboard.putNumber("Arm Master", current3);

   

   
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.m_arm, RobotContainer.m_moveArm);
  }
}
