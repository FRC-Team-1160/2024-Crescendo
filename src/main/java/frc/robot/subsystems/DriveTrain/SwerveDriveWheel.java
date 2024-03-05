package frc.robot.subsystems.DriveTrain;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
// import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveDriveWheel {
    public PIDController directionController;
    public TalonFX steerMotor;
    public TalonFX driveMotor;
    public CANcoder steerSensor;
    public double angle;
    public double speed;
    public double distance;
    public PIDController m_anglePID;
    public PIDController m_speedPID;

    //public TalonFXSensorCollection directionSensor;
    double kFF, accumulator, maxA;

    public SwerveDriveWheel(TalonFX steerMotor, CANcoder steerSensor, TalonFX driveMotor)
    {
        this.driveMotor = driveMotor;
        this.steerSensor = steerSensor;
        this.steerMotor = steerMotor;
        maxA = 100;
        accumulator = 0;
        kFF = 0.05;
        distance = 0;
        m_anglePID = new PIDController(0.5, 0.0, 0.001);

        m_anglePID.enableContinuousInput(-0.5, 0.5);
    }

    public void set(double angle, double speed)
    {
        //***NEW API, DOUBLE CHECK OR TEST***
        double a = m_anglePID.calculate(getAngle(), angle);
        SmartDashboard.putNumber("MANUAL TARGET", a);
        //double v = m_speedPID.calculate(getSpeed(), speed);
        if (Math.abs(getAngle() - angle) > 0.005){
            steerMotor.set(-a);
        } else {
            steerMotor.set(0);
        }
        driveMotor.set(speed);

        // steerMotor.setControl(new PositionDutyCycle(angle)); //CHECK UNITS 
        // driveMotor.setControl(new VelocityDutyCycle(speed)); //rots/sec
        this.angle = angle;
        this.speed = speed;
        this.distance += speed * 6;
    }

    public double getPosition(){
        if (RobotBase.isReal()){
            return driveMotor.getRotorPosition().getValue();
        } else {    
            return distance;
        }
    }

    public double getSpeed(){
        if (RobotBase.isReal()){
            return driveMotor.getRotorVelocity().getValue();
        } else {
            return speed;
        }
    }

    public double getAngle(){
        if (RobotBase.isReal()){
            double v = (steerSensor.getAbsolutePosition().getValue() % 1d);
            if (v < -0.5){
                v += 1.0;
            }
            if (v > 0.5){
                v -= 1.0;
            }
            // if (v < 0){
            //     v += 1;
            // }
            return v;
        } else {
            // double v = (angle % 1d);
            // if (v < -0.5){
            //     v += 1.0;
            // }
            // if (v > 0.5){.


            //     v -= 1.0;
            // }
            // return v;
            return angle;
        }
    }
    
}