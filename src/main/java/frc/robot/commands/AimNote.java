
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Vision.Vision;

public class AimNote extends Command {
  /** Creates a new SwerveDrive. */
  private double x;
  private double y;
  DriveTrain m_drive;
  Vision m_vision;

  public AimNote(DriveTrain m_drive, Vision m_vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_vision);
    this.m_drive = m_drive;
    this.m_vision = m_vision;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){

    double[] inputs = m_drive.inputSpeeds();
    x = inputs[0];
    y = inputs[1];

    SmartDashboard.putNumber("Forward", x);
    SmartDashboard.putNumber("Sideways", y);
    Pose3d note = m_vision.tracked_note;
    if (note != null){
      m_drive.aimReverse(x, y, note.getX(), note.getY());
    }

    // if (m_mainStick.getRawButton(1) || RobotBase.isSimulation()){
    //   m_drive.aimSwerveDrive(x, y, 0.5, 5.5);
    // } else {
    //   m_drive.setSwerveDrive(0, 0, 0);
    // }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
    