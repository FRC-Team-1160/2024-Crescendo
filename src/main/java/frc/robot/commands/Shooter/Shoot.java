package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter m_shooter;
    double speed;

    public Shoot(Shooter m_shooter, double speed){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
        this.speed = speed;
    }

    @Override
    public void initialize(){
        m_shooter.setSpeed(0.5);
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }



}