// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// THIS CLASS CONTAINES OUR SETPOINTS FOR ALL OF OUR POSITIONS FOR THE 
//    SHOULDER, EXTENDER, AND WRIST.
//
// THE VALUES ARE SORTED BY POSITION TO MAKE THEM EASIER TO FIND AND TUNE

public class PositionSetpoints {

    // Min position for each subsystem
    public static final double SHOULDER_POSITION_MIN = 0; // Min position of the arm

    // Max position for every subsystem
    public static final double SHOULDER_POSITION_MAX = 92500; // Max position of the arm

    // Position for the Shoulder to be set to move everything else
    public static final double SHOULDER_POSITION_SAFE_TO_EXTEND = 31000; // Safe distance where can extend the rest of
                                                                         // the arm

    // STOW Position for each Subsystem
    public static final double SHOULDER_POSITION_STOW = 0;
    public static final double EXTENDER_POSITION_STOW = 0;
    public static final double WRIST_POSITION_STOW = 0;

    // PICKUP_CONE Position for each subsystem
    public static final double SHOULDER_POSITION_PICKUP_CONE = 37500;
    public static final double EXTENDER_POSITION_PICKUP_CONE = 22.0;
    public static final double WRIST_POSITION_PICKUP_CONE = 33.5;  //  Old value:  42.75;

    // PICKUP_CUBE Position for each subsystem
    public static final double SHOULDER_POSITION_PICKUP_CUBE = 38000;
    public static final double EXTENDER_POSITION_PICKUP_CUBE = 37.0;
    public static final double WRIST_POSITION_PICKUP_CUBE = 33.5;  //  Old Value:  42.47;

    // Position for each LOW_CONE_NODE subsystem
    public static final double SHOULDER_POSITION_LOW_CONE_NODE = 42500;
    public static final double EXTENDER_POSITION_LOW_CONE_NODE = 2.60;
    public static final double WRIST_POSITION_LOW_CONE_NODE = 44.86;  // Old Value:  56.07;

    // MID_CONE_Position for each subsystem
    public static final double SHOULDER_POSITION_MID_CONE_NODE = 77500;
    public static final double EXTENDER_POSITION_MID_CONE_NODE = 1.83;
    public static final double WRIST_POSITION_MID_CONE_NODE = 42.46;  // Old Value:  53.07;

    // HIGH_CONE_NODE_Position for each subsystem
    public static final double SHOULDER_POSITION_HIGH_CONE_NODE = 92500;
    public static final double EXTENDER_POSITION_HIGH_CONE_NODE = 49.05;
    public static final double WRIST_POSITION_HIGH_CONE_NODE = 38.46;  //  Old Value:  48.07;

    // LOW_CUBE_NODE_Position for each subsystem
    public static final double SHOULDER_POSITION_LOW_CUBE_NODE = 42500;
    public static final double EXTENDER_POSITION_LOW_CUBE_NODE = 9.07;
    public static final double WRIST_POSITION_LOW_CUBE_NODE = 11.16;  //Old Value: 13.95;

    // MID_CUBE_NODE_Position for each subsystem
    public static final double SHOULDER_POSITION_MID_CUBE_NODE = 57500;
    public static final double EXTENDER_POSITION_MID_CUBE_NODE = 23.00;
    public static final double WRIST_POSITION_MID_CUBE_NODE = 21.66;  //  Old Value:  27.07;

    // HIGH_CUBE_NODE Position for each subsystem
    public static final double SHOULDER_POSITION_HIGH_CUBE_NODE = 80000;
    public static final double EXTENDER_POSITION_HIGH_CUBE_NODE = 75.43;
    public static final double WRIST_POSITION_HIGH_CUBE_NODE = 28.00;  // Old Value:  35.00

    // HUMAN_PLAYER_STATION Position for each subsystem
    public static final double SHOULDER_POSITION_HUMAN_PLAYER_STATION = 20500;
    public static final double EXTENDER_POSITION_HUMAN_PLAYER_STATION = 0.0;
    public static final double WRIST_POSITION_HUMAN_PLAYER_STATION = 1.0;  //  Old Value:  8.5;

}
