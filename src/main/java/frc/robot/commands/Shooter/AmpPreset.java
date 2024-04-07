package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.Transport;

public class AmpPreset extends Command {
    Shooter m_shooter;
    double speed;
    Transport m_transport;

    public AmpPreset(Shooter m_shooter){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
    }

    @Override
    public void initialize(){
        m_shooter.setpoint = 0.16;
        SmartDashboard.putNumber("Shooter Pitch", 0.16);
        m_shooter.setSpeed(0.3);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putNumber("Shooter Pitch", 0.0);
        m_shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }



}