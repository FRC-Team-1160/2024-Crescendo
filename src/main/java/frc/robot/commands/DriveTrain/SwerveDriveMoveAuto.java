package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public Double target_a;

    public double start_x; 
    public double start_y;

    public double dist;

    public Timer m_timer;

    public boolean red;

    public double kP;

    DriveTrain m_drive;

    public SwerveDriveMoveAuto(DriveTrain m_drive, double x, double y, Double a){
        addRequirements(m_drive);
        this.m_drive = m_drive;
        target_x = x;
        target_y = y;
        target_a = a;
        m_profile = null;
        kP = 0.003;
    }
    public SwerveDriveMoveAuto(DriveTrain m_drive, double x, double y, Double a, TrapezoidProfile.Constraints constraints, double p){
        addRequirements(m_drive);
        this.m_drive = m_drive;
        target_x = x;
        target_y = y;
        target_a = a;
        m_profile = new TrapezoidProfile(constraints);
        kP = p;
    }

    @Override
    public void initialize(){
        if (m_drive.isRed) {
            target_x = 16.54 - target_x;
            if (target_a != null){
                target_a = 180 - target_a;
            }
        }
        if (m_profile == null) m_profile = new TrapezoidProfile(Constants.Auto.kVelocityControllerConstraints);
        start_x = m_drive.odomPose.getX();
        start_y = m_drive.odomPose.getY();
        dist = Math.sqrt(Math.pow(target_x - start_x, 2) + Math.pow(target_y - start_y, 2));
        x_pid = new PIDController(0.3, 0, 0);
        y_pid = new PIDController(0.3, 0, 0);
        a_pid = new ProfiledPIDController(kP, 0, 0, Constants.Auto.kThetaControllerConstraints);
        a_pid.enableContinuousInput(-180, 180);
        a_pid.setTolerance(5);
        a_pid.reset(m_drive.getGyroAngle());
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
        double a = (target_a == null) ? 0 : a_pid.calculate(m_drive.getGyroAngle(), target_a);

        m_drive.setSwerveDrive(x, y, a);
        if (target_a != null) SmartDashboard.putNumber("target_a", target_a); 
        SmartDashboard.putNumber("a_pid", a);
        SmartDashboard.putNumber("a_diff", a_pid.getPositionError());

    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return (Math.sqrt(Math.pow(m_drive.odomPose.getX() - target_x, 2) + Math.pow(m_drive.odomPose.getY() - target_y, 2)) < 0.1);
    }

}
