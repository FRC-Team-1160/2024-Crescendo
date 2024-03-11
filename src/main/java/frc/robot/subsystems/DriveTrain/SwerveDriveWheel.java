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
        m_anglePID = new PIDController(1.2, 0.0, 0.005);

        m_anglePID.enableContinuousInput(-0.5, 0.5);
    }

    public void set(double angle, double speed)
    {
        double a = m_anglePID.calculate(getAngle(), angle);
        SmartDashboard.putNumber("MANUAL TARGET", a);
        if (Math.abs(getAngle() - angle) > 0.005){
            steerMotor.set(-a);
        } else {
            steerMotor.set(0);
        }
        driveMotor.set(speed);

        this.angle = angle;
        this.speed = speed;
        this.distance += speed;
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

            return v;
        } else {
            return angle;
        }
    }
    
}