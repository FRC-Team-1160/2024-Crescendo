package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AngleShooter extends Command {
    Shooter m_shooter;
    double angle;

    public AngleShooter(Shooter m_shooter, double angle){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
        this.angle = angle;
    }

    @Override
    public void initialize(){
        m_shooter.setpoint = angle;
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