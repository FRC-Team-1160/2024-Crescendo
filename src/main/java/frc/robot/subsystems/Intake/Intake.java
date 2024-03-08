package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

public class Intake extends SubsystemBase{
    private static Intake m_instance;

    public Compressor m_compressor;
    public DoubleSolenoid m_solenoid;
    public CANSparkMax m_feedMotor;

    public int solenoidState;
    public boolean motorState;

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
        m_feedMotor = new CANSparkMax(PortConstants.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);

        solenoidState = 0;
        motorState = false;
        m_transport = Transport.getInstance();
    }

    public void setSolenoid(int state){
        solenoidState = state;
        System.out.println(state);
        switch (state) {
            case -1 ->
                m_solenoid.set(DoubleSolenoid.Value.kReverse);
            case 0 ->
                m_solenoid.set(DoubleSolenoid.Value.kOff);
            case 1 ->
                m_solenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public int toggleSolenoid(){
        setSolenoid(-solenoidState);
        return(solenoidState);
    }

    public void setMotor(boolean state){
        motorState = state;
        if (state){
            m_feedMotor.set(0.9);
        } else {
            m_feedMotor.set(0);
        }
    }

    public void toggleMotor(){
        motorState = !motorState;
        setMotor(motorState);
    }

    @Override
    public void periodic(){
        // SmartDashboard.putString("Solenoid State", m_solenoid.get().toString());
    }

}
