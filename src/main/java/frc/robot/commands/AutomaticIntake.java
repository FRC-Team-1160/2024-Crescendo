
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Vision.Vision;

public class AutomaticIntake extends Command {
  /** Creates a new SwerveDrive. */
  private double x;
  private double y;
  DriveTrain m_drive;
  Vision m_vision;
  Pose2d note_pos;

  ProfiledPIDController d_pid;
  PIDController x_pid, y_pid;


  public AutomaticIntake(DriveTrain m_drive, Vision m_vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_vision);
    this.m_drive = m_drive;
    this.m_vision = m_vision;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    d_pid = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(0.5, 1));
    note_pos = m_vision.tracked_note.toPose2d();
    x_pid = new PIDController(0.5, 0, 0);
    y_pid = new PIDController(0.5, 0, 0);
    d_pid.setGoal(new TrapezoidProfile.State(0, 0));

  }

  @Override
  public void execute(){
    Pose2d pose = m_drive.odomPose;
    // double x = x_pid.calculate(.m_pose);
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
    