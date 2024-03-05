package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter m_shooter;
    double speed;
    Timer m_timer;

    public Shoot(Shooter m_shooter, double speed){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
        this.speed = speed;
    }

    @Override
    public void initialize(){
        m_shooter.setSpeed(speed);
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.setSpeed(0.2);

    }

    @Override
    public boolean isFinished(){
        return (m_timer.get() > 1.0);
    }



}