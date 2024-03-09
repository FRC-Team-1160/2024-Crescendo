package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Transport;

public class OuttakeNote extends Command{
    Intake m_intake;
    Transport m_transport;
    Shooter m_shooter;
    Timer m_timer;

    public OuttakeNote(Intake m_intake, Transport m_transport, Shooter m_shooter){
        addRequirements(m_intake, m_transport);
        this.m_intake = m_intake;
        this.m_transport = m_transport;
        this.m_shooter = m_shooter;
    }

    @Override
    public void initialize(){
        m_intake.m_solenoid.set(DoubleSolenoid.Value.kForward);
        m_intake.m_feedMotor.set(-0.9);
        m_transport.setWheels(-1);
        m_transport.belt.set(0.25);
        m_shooter.setSpeed(-0.25);
        m_timer = new Timer();
    }

    @Override
    public void execute(){
        if (m_transport.noteStored && m_timer.get() == 0.0){
            m_timer.start();
        }
    }

    @Override
    public void end(boolean interrupted){
        if (!new JoystickButton(new Joystick(Constants.IO.RIGHT_BOARD_PORT), Constants.IO.Board.Right.UP_DOWN_INTAKE).getAsBoolean()) m_intake.m_solenoid.set(DoubleSolenoid.Value.kReverse);
        m_intake.m_feedMotor.set(0);
        m_transport.setWheels(0);
        m_transport.belt.set(0);
        m_shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return (m_timer.hasElapsed(0.1));
    }
}
