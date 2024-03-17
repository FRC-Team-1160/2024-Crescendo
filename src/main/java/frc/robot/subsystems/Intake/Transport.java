package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Port;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Transport extends SubsystemBase {

    private static Transport m_instance;

    public CANSparkMax leftWheel;
    public CANSparkMax rightWheel;
    public CANSparkMax belt;
    public AnalogPotentiometer ultrasonic;

    public boolean noteStored;

    public Timer refresh;

    public ColorSensorV3 m_colorSensor;

    public static Transport getInstance(){
        if (m_instance == null){
            m_instance = new Transport();
        }
        return m_instance;
    }

    public Transport(){

        leftWheel = new CANSparkMax(Port.TRANSPORT_LEFT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        rightWheel = new CANSparkMax(Port.TRANSPORT_RIGHT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        belt = new CANSparkMax(Port.TRANSPORT_BELT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        noteStored = false;
        refresh = new Timer();
        refresh.start();
    }
    
    public void setWheels(int state){
        leftWheel.set(-0.2*state);
        rightWheel.set(0.2*state);
    }

    @Override
    public void periodic(){
        if (refresh.hasElapsed(0.1)) {
            refresh.restart();
            int prox = m_colorSensor.getProximity();
            SmartDashboard.putNumber("Color Sensor Prox", prox);
            noteStored = (prox > 200.0); //nothing ~100, note ~350
            SmartDashboard.putBoolean("Note Stored", noteStored);
        }
    }
}
