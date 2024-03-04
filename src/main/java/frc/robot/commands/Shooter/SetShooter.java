package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
    Shooter m_shooter;
    double speed;

    public SetShooter(Shooter m_shooter){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
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