package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Transport;

public class IntakeNote extends Command{
    Intake m_intake;
    Transport m_transport;

    public IntakeNote(Intake m_intake, Transport m_transport){
        addRequirements(m_intake, m_transport);
        this.m_intake = m_intake;
        this.m_transport = m_transport;
    }

    @Override
    public void initialize(){
        m_intake.setSolenoid(1);
        m_intake.setMotor(true);
        m_transport.setWheels(true);
        m_transport.setBelt(1);
    }

    @Override
    public void end(boolean interrupted){
        m_intake.setSolenoid(-1);
    }

    @Override
    public boolean isFinished(){
        return m_transport.noteStored;
    }
}
