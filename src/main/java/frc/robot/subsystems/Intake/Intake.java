package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.Port;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

public class Intake extends SubsystemBase{
    private static Intake m_instance;

    public Compressor m_compressor;
    public DoubleSolenoid m_solenoid;
    public CANSparkMax m_feedMotor;

    public Transport m_transport;

    public static Intake getInstance(){
        if (m_instance == null){
            m_instance = new Intake();
        }
        return m_instance;
    }

    private Intake(){
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        m_feedMotor = new CANSparkMax(Port.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        m_transport = Transport.getInstance();
        m_solenoid.set(!new JoystickButton(new Joystick(Constants.IO.RIGHT_BOARD_PORT), Constants.IO.Board.Right.UP_DOWN_INTAKE).getAsBoolean() ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    @Override
    public void periodic(){
    }

}
