package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IncrementShooter extends Command {
    Shooter m_shooter;
    double amount;
    boolean run;

    public IncrementShooter(Shooter m_shooter, double amount){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
        this.amount = amount;
    }

    @Override
    public void initialize(){
        double speed = Math.max(Math.min(m_shooter.speed + amount, 1), 0);
        m_shooter.setSpeed(speed);
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