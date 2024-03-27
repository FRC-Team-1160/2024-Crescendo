package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.Transport;

public class Shoot extends Command {
    Shooter m_shooter;
    double speed;
    Timer m_timer;
    Transport m_transport;

    public Shoot(Shooter m_shooter, Transport m_transport){
        addRequirements(m_shooter, m_transport);
        this.m_shooter = m_shooter;
        this.m_transport = m_transport;
    }

    @Override
    public void initialize(){
        m_transport.belt.set(-0.60);
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){
        m_transport.belt.set(0);
    }

    @Override
    public boolean isFinished(){
        return (m_timer.hasElapsed(1.0));
    }



}