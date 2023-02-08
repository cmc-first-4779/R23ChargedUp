// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.StaticConstants;

/**
 * These are values from the Limelight Network API table...
 * These NEVER change and so we are leaving them in a seperate class so that
 * RobotMap is cleaner and only contains values that we would be changing to
 * tune the robot..
 * 
 * We will leave the changing Limelight values STILL in RobotMap...
 */

public class LimelightConstants {
    /************************************************************************/
    /*************** LIMELIGHT / VISION CONSTANTS *****************************/
    /************************************************************************/
    //
    // LIMELIGHT Camera Modes
    public static final double LIMELIGHT_CAMMODE_VISION = 0; // Vision Processing Mode = 0
    public static final double LIMELIGHT_CAMMODE_DRIVER = 1; // Driver Mode = 1
    // LIMELIGHT LED Modes
    public static final double LIMELIGHT_LEDMODE_PIPELINE_DEFAULT = 0; // Uses the Default LED Mode for the Pipeline
    public static final double LIMELIGHT_LEDMODE_OFF = 1; // Force LED Mode Off
    public static final double LIMELIGHT_LEDMODE_BLINK = 2; // Force LED Mode Blink
    public static final double LIMELIGHT_LEDMODE_ON = 3; // Force LED Mode On
    // LIMELIGHT TV Value when there is no target
    public static final double LIMELIGHT_NO_TARGET = 0.0; // TV Value when the Limelight doesn't see its target
    public static final double LIMELIGHT_HAS_TARGET = 1.0; // TV Value when the Limelight has a target
    // Camera Pixel Count and Field of View Constants
    public static final double LIMELIGHT_X_PIXEL_COUNT = 320; // pixels
    public static final double LIMELIGHT_Y_PIXEL_COUNT = 240; // pixels
    public static final double LIMELIGHT_X_FOV = 59.6; // degrees
    public static final double LIMELIGHT_Y_FOV = 45.7; // degrees

    // Limelight Streaming Modes
    public static final double LIMELIGHT_STREAMING_STANDARD_MODE = 0; // Side-by-side streams if a webcam is attached
    public static final double LIMELIGHT_PiP_MAIN_MODE = 1; // Secondary camera stream is placed in the lower right
                                                            // camera
    public static final double LIMELIGHT_PiP_SECONDARY_MODE = 2; // primary camera is placed in lower right of secondary



}
