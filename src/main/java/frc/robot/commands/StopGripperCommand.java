package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;

<<<<<<< HEAD
public class StopGripperCommand extends CommandBase{
    GripperSubsystem m_gripperSubsystem;
    double currentEncoderPosition;
    /** Creates a new OpenGripperCommand. */
    public StopGripperCommand(GripperSubsystem gripperSubsystem) {
      m_gripperSubsystem = gripperSubsystem;
      // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_gripperSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      currentEncoderPosition = m_gripperSubsystem.getEncoder();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      currentEncoderPosition = m_gripperSubsystem.getEncoder(); 
      m_gripperSubsystem.openGripper(2);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_gripperSubsystem.stopGripper(); 
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return currentEncoderPosition <= Constants.GRIPPER_ENCODER_MIN; 
    }
  
  
}
    
=======
public class StopGripperCommand extends CommandBase {
        GripperSubsystem m_gripperSubsystem; 
        double currentEncoderPosition;
        /** Creates a new closeGripperCommand. */
        public StopGripperCommand(GripperSubsystem gripperSubsystem) {
          m_gripperSubsystem = gripperSubsystem;
          // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_gripperSubsystem);
        }
      
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
          currentEncoderPosition = m_gripperSubsystem.getEncoder();
        }
      
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
          currentEncoderPosition = m_gripperSubsystem.getEncoder(); 
      m_gripperSubsystem.stopGripper();
        }

        // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentEncoderPosition <= Constants.GRIPPER_ENCODER_MIN; 
  }
}
>>>>>>> 4f0a8153fa07c651bf2825977ab508acde938e59

