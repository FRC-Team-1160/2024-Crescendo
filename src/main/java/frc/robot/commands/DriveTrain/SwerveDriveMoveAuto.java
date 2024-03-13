package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class SwerveDriveMoveAuto extends Command {
    
    public ProfiledPIDController m_pid;

    public double target_x;
    public double target_y;

    public double start_x; 
    public double start_y;

    public double dist;

    DriveTrain m_drive;

    public SwerveDriveMoveAuto(DriveTrain m_drive, double x, double y){
        addRequirements(m_drive);
        this.m_drive = m_drive;
        this.target_x = x;
        this.target_y = y;
    }

    @Override
    public void initialize(){
        m_pid = new ProfiledPIDController(0.1, 0, 0, Constants.Auto.kVelocityControllerConstraints);
        start_x = m_drive.odomPose.getX();
        start_y = m_drive.odomPose.getY();
        dist = Math.sqrt(Math.pow(target_x - start_x, 2) + Math.pow(target_y - start_y, 2));
        System.out.println(dist);
        m_pid.setGoal(dist);
        m_pid.setTolerance(0.1, 0.05);
        System.out.println("CommandInit");
    }

    @Override
    public void execute(){
        double c = Math.sqrt(Math.pow(m_drive.odomPose.getX() - start_x, 2) + Math.pow(m_drive.odomPose.getY() - start_y, 2));
        double d = m_pid.calculate(c);
        double x = (target_x - start_x) * d / dist;
        double y = (target_y - start_y) * d / dist;
        m_drive.setSwerveDrive(x, y, 0);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return m_pid.atGoal();
    }

}
