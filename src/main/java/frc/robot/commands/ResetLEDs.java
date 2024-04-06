package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.PWM;

public class ResetLEDs extends Command {

    Shooter m_shooter;
    PWM m_pwm;

    public ResetLEDs(Shooter m_shooter){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
    }

    @Override
    public void initialize(){
        m_pwm = m_shooter.m_pwm;
        m_pwm.setPulseTimeMicroseconds(2125);
        m_pwm.setPulseTimeMicroseconds(1745);
        m_pwm.setPulseTimeMicroseconds(1755);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}