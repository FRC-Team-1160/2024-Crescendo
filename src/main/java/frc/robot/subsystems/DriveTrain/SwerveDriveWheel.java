package frc.robot.subsystems.DriveTrain;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
// import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class SwerveDriveWheel {
    public PIDController directionController;
    public TalonFX steerMotor;
    public TalonFX driveMotor;
    public CANcoder steerSensor;
    public double distance;
    public SwerveModuleState sim_state;
    public PIDController m_anglePID;

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

        var drive_configs = new Slot0Configs();
        drive_configs.kP = 0.05;
        drive_configs.kV = 0.12;
        drive_configs.kS = 0.10;
        driveMotor.getConfigurator().apply(drive_configs);

        m_anglePID = new PIDController(1.4, 0.0, 0.005);
        m_anglePID.enableContinuousInput(-0.5, 0.5);
    }

    public void set(SwerveModuleState state)
    {
        double angle = state.angle.getRotations();
        double a = m_anglePID.calculate(getAngle().getRotations(), angle);
        if (Math.abs(getAngle().getRotations() - angle) > 0.005){
            steerMotor.set(-a);
        } else {
            steerMotor.set(0);
        }

        driveMotor.setControl(new VelocityVoltage(state.speedMetersPerSecond / Constants.Swerve.WHEEL_DIAMETER * Constants.Swerve.GEAR_RATIO));

        sim_state = state;
        this.distance += state.speedMetersPerSecond;
    }

    public double getPosition(){
        if (RobotBase.isReal()){
            return driveMotor.getRotorPosition().getValue() * Constants.Swerve.WHEEL_DIAMETER / Constants.Swerve.GEAR_RATIO;
        } else {    
            return distance;
        }
    }

    public double getSpeed(){
        if (RobotBase.isReal()){
            return driveMotor.getRotorVelocity().getValue() * Constants.Swerve.WHEEL_DIAMETER / Constants.Swerve.GEAR_RATIO;
        } else {
            return sim_state.speedMetersPerSecond;
        }
    }

    public Rotation2d getAngle(){
        if (RobotBase.isReal()){
            double v = (steerSensor.getAbsolutePosition().getValue() % 1d);
            if (v < -0.5){
                v += 1.0;
            }
            if (v > 0.5){
                v -= 1.0;
            }
            return Rotation2d.fromRotations(v);
        } else {
            return Rotation2d.fromRotations(sim_state.speedMetersPerSecond);
        }
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getSpeed(), getAngle());
    }
    
}