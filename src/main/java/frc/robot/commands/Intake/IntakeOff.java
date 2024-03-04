package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Transport;

public class IntakeOff extends Command{
    Intake m_intake;
    Transport m_transport;

    public IntakeOff(Intake m_intake){
        addRequirements(m_intake);
        this.m_intake = m_intake;
        this.m_transport = m_intake.m_transport;
    }

    @Override
    public void initialize(){
        m_intake.setSolenoid(-1);
        m_intake.setMotor(false);
        m_transport.setWheels(false);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
