package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Transport;

public class IntakeToggle extends Command{
    Intake m_intake;
    Transport m_transport;

    public IntakeToggle(Intake m_intake){
        addRequirements(m_intake);
        this.m_intake = m_intake;
    }

    @Override
    public void initialize(){
        m_intake.toggleSolenoid();

    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
