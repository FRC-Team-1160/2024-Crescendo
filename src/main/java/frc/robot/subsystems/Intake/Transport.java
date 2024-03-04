package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PortConstants;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Transport extends SubsystemBase {

    private static Transport m_instance;

    public CANSparkMax leftWheel;
    public CANSparkMax rightWheel;
    public CANSparkMax belt;

    public boolean beltState;
    public boolean wheelsState;

    public AnalogPotentiometer ultrasonic;

    public boolean noteStored;

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
        
        ultrasonic = new AnalogPotentiometer(PortConstants.TRANSPORT_ULTRASONIC);

        noteStored = false;

        beltState = false;
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

    public void setBelt(boolean state){
        beltState = state;
        if (state){
            belt.set(-0.5);
        } else {
            belt.set(0);
        }
    }

    public void toggleBelt(){
        beltState = !beltState;
        setBelt(beltState);
    }

    @Override
    public void periodic(){
        noteStored = (ultrasonic.get() < 10.0); //CHECK VALUE
    }
}
