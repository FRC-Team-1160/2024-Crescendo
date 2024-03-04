package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeUp extends Command{
    Intake m_intake;

    public IntakeUp(Intake m_intake){
        addRequirements(m_intake);
        this.m_intake = m_intake;
    }

    @Override
    public void initialize(){
        m_intake.setSolenoid(-1);

    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
