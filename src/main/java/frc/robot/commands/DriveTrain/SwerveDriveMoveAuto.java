package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class SwerveDriveMoveAuto extends Command {
    
    public PIDController x_pid;
    public PIDController y_pid;

    public TrapezoidProfile m_profile;

    public ProfiledPIDController a_pid;

    public double target_x;
    public double target_y;
    public double target_a;

    public double start_x; 
    public double start_y;

    public double dist;

    public Timer m_timer;

    DriveTrain m_drive;

    public SwerveDriveMoveAuto(DriveTrain m_drive, double x, double y, double a){
        addRequirements(m_drive);
        this.m_drive = m_drive;
        target_x = x;
        target_y = y;
        target_a = a;
    }

    public SwerveDriveMoveAuto(DriveTrain m_drive, double x, double y){
        addRequirements(m_drive);
        this.m_drive = m_drive;
        this.target_x = x;
        this.target_y = y;
        target_a = 999;
    }

    @Override
    public void initialize(){
        m_profile = new TrapezoidProfile(Constants.Auto.kVelocityControllerConstraints);
        start_x = m_drive.odomPose.getX();
        start_y = m_drive.odomPose.getY();
        dist = Math.sqrt(Math.pow(target_x - start_x, 2) + Math.pow(target_y - start_y, 2));
        System.out.println(dist);
        x_pid = new PIDController(0.1, 0, 0);
        y_pid = new PIDController(0.1, 0, 0);
        a_pid = new ProfiledPIDController(0.0001, 0, 0, Constants.Auto.kThetaControllerConstraints);
        a_pid.enableContinuousInput(-180, 180);
        m_timer = new Timer();
        m_timer.start();

    }

    @Override
    public void execute(){
        double pos = m_profile.calculate(m_timer.get(),
            new TrapezoidProfile.State(Math.sqrt(Math.pow(m_drive.odomPose.getX() - start_x, 2) + Math.pow(m_drive.odomPose.getY() - start_y, 2)), 0),
            new TrapezoidProfile.State(dist, 0)
        ).position;
        double x = x_pid.calculate(m_drive.odomPose.getX(), start_x + (target_x - start_x) * pos/dist);
        double y = y_pid.calculate(m_drive.odomPose.getY(), start_y + (target_y - start_y) * pos/dist);
        double a = 0;
        if (target_a != 999){
            a = -a_pid.calculate(m_drive.getGyroAngle(), target_a);
        }
        m_drive.setSwerveDrive(x, y, a);

        System.out.println(target_a);
        System.out.println(m_drive.getGyroAngle());

    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return (Math.sqrt(Math.pow(m_drive.odomPose.getX() - target_x, 2) + Math.pow(m_drive.odomPose.getY() - target_y, 2)) < 0.1);
    }

}
