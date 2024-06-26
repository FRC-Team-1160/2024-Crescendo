package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Transport;

public class IntakeNote extends Command{
    Intake m_intake;
    Transport m_transport;
    Timer m_timer;

    public IntakeNote(Intake m_intake, Transport m_transport){
        addRequirements(m_intake, m_transport);
        this.m_intake = m_intake;
        this.m_transport = m_transport;
    }

    @Override
    public void initialize(){
        m_intake.m_solenoid.set(DoubleSolenoid.Value.kForward);
        m_intake.m_feedMotor.set(0.75); //0.75
        m_transport.setWheels(1);
        m_transport.belt.set(-0.25);
        m_timer = new Timer();
        m_intake.blinkin.set(0.15);
    }

    @Override
    public void execute(){
        if (m_transport.noteStored){
            m_timer.start();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_intake.m_solenoid.set(m_intake.solenoid_default);
        m_intake.m_feedMotor.set(0);
        m_transport.setWheels(0);
        m_transport.belt.set(0);
        if (m_transport.noteStored){
            m_intake.blinkin.set(0.85);
        } else {
            m_intake.blinkin.set(0.93);
        }
    }

    @Override
    public boolean isFinished(){
        return (m_timer.hasElapsed(0.1));
    }
}
