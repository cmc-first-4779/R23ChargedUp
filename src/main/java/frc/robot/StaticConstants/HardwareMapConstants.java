// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StaticConstants;

/** Add your docs here. */
public class HardwareMapConstants {
    // ***************** CAN ADDRESSES *******************************************/
    // Any Victors that are slaved to Talons have a CAN address of 10 + the TALON
    // address that it is slaved to.
    public static final int CAN_ADDRESS_GRIPPER = 2; //  Gripper Motor 
    public static final int CAN_ADDRESS_EXTENDER = 3; // Extender Motor
    public static final int CAN_ADDRESS_WRIST_MOTOR_LEFT = 4; // Left Wrist Motor
    public static final int CAN_ADDRESS_WRIST_MOTOR_RIGHT = 5; // Right Wrist Motor

    public static final int DRIVERSTICK_USB_PORT = 0;
    public static final int OPERSTICK_USB_PORT = 1;
    
    public static final int PWM_PORT_BLING = 0;
}
