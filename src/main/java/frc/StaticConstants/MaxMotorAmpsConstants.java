// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.StaticConstants;

/**
 * This is where we store the maximum recommended amperage for each of
 * our motors from Vex Robotics.
 * https://motors.vex.com/
 * 
 * We will call this class as we initiatize our motor controllers.
 */

public class MaxMotorAmpsConstants {
    // Falcon 500 Current Limit settings
    public static final double MAX_AMPS_STATOR_LIMIT_FALCON500 = 30;
    public static final double MAX_AMPS_STATOR_TRIGGER_FALCON500 = 40;
    // public static final double MAX_AMPS_STATOR_LIMIT_FALCON500 = 15;  //To protect falcons during testing
    // public static final double MAX_AMPS_STATOR_TRIGGER_FALCON500 = 20; //To protect falcons during testing
    public static final double MAX_SECS_STATOR_THRESHOLDTIME_FALCON500 = 200;
    // Neo 550 Current Limit Settings
    public static final int MAX_AMPS_STATOR_LIMIT_NEO550 = 17;
    public static final int MAX_AMPS_STATOR_TRIGGER_NEO550 = 17;
    public static final double MAX_SECS_STATOR_THRESHOLDTIME_NEO550 = 60;
    // 775 PRO Current Limit Settings
    public static final int MAX_AMPS_STATOR_LIMIT_775PRO = 30;
    public static final int MAX_AMPS_STATOR_TRIGGER_775PRO = 40;
    public static final int MAX_SECS_STATOR_THRESHOLDTIME_775PRO = 60;
    // NEO Current Limit Settings
    public static final int MAX_AMPS_STATOR_LIMIT_NEO = 40;
    public static final int MAX_AMPS_STATOR_TRIGGER_NEO = 50;
    public static final int MAX_AMPS_STATOR_THRESHOLDTIME_NEO = 60;
    // We will figure these ones out later..
    public static final double MAX_AMPS_CIM = 0;
    public static final double MAX_AMPS_MINICIM = 0;
    public static final double MAX_AMPS_REDLINE = 0;
    public static final double MAX_AMPS_BAG = 0;

}
