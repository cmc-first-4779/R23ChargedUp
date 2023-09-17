// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorPickupSCG extends SequentialCommandGroup {
    // Declare our Subsystems
    ShoulderSubsystem shoulder;
    ExtenderSubsystem extender;
    WristSubsystem wrist;

  /** Creates a new FloorPickSCG. */
  public FloorPickupSCG(boolean cubeMode, ShoulderSubsystem shoulder, ExtenderSubsystem extender, WristSubsystem wrist) {
    // Set our local variables to the pass-thru values
    this.shoulder = shoulder;
    this.extender = extender;
    this.wrist = wrist;

    System.out.println("FloorPickupSCG with cubeMode: " + cubeMode);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  if (cubeMode) {
    addCommands(
      new SafeSetToPositionSCG("PICKUP_CUBE", shoulder, extender, wrist)
    );
  }
  else {
    addCommands (
      new SafeSetToPositionSCG("PICKUP_CONE", shoulder, extender, wrist)
    );
  }    
}

}
