package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import edu.wpi.first.math.geometry.Pose2d;

public class AimShooter extends Command {

    private Pose2d pose;
    Shooter m_shooter;
    DriveTrain m_drive;

    public AimShooter(Shooter m_shooter, DriveTrain m_drive){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
    }

    @Override
    public void initialize(){
        pose = m_drive.odomPose;
    }

    @Override
    public void execute(){
        double angle = Math.atan2(1.9, Math.pow(pose.getX() - 0.5, 2) + Math.pow(pose.getY() - Constants.Field.SPEAKER_Y, 2));
        m_shooter.setpoint = angle;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
