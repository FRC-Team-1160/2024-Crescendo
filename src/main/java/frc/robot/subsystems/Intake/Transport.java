package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PortConstants;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Transport extends SubsystemBase {

    private static Transport m_instance;

    public CANSparkMax leftWheel;
    public CANSparkMax rightWheel;
    public CANSparkMax belt;

    public int beltState;
    public boolean wheelsState;

    public AnalogPotentiometer ultrasonic;

    public boolean noteStored;

    public ColorSensorV3 m_colorSensor;

    public static Transport getInstance(){
        if (m_instance == null){
            m_instance = new Transport();
        }
        return m_instance;
    }

    public Transport(){

        leftWheel = new CANSparkMax(PortConstants.TRANSPORT_LEFT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        rightWheel = new CANSparkMax(PortConstants.TRANSPORT_RIGHT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        belt = new CANSparkMax(PortConstants.TRANSPORT_BELT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        
        m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        noteStored = false;
        beltState = 0;
        wheelsState = false;
    }

    public void setWheels(boolean state){
        wheelsState = state;
        if (state){
            leftWheel.set(-0.2);
            rightWheel.set(0.2);
        } else {
            leftWheel.set(0);
            rightWheel.set(0);
        }
    }

    public void toggleWheels(){
        wheelsState = !wheelsState;
        setWheels(wheelsState);
    }

    public void setBelt(int state){
        beltState = state;
        if (state == 1){
            belt.set(-0.3);
        } else if (state == 2){
            belt.set(-0.6);
        } else {
            belt.set(0.0);
        }

    }

    public void toggleBelt(){
        if (beltState == 0){
            beltState = 1;
        } else {
            beltState = 0;
        }
        setBelt(beltState);
    }

    @Override
    public void periodic(){
        int prox = m_colorSensor.getProximity();
        SmartDashboard.putNumber("Color Sensor Prox", prox);
        noteStored = (prox > 200.0); //nothing = 120, note ~350
        SmartDashboard.putNumber("Belt State", beltState);
    }
}
