package frc.robot.subsystems.DriveTrain;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
// import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveDriveWheel {
    public PIDController directionController;
    public TalonFX steerMotor;
    public TalonFX driveMotor;
    public CANcoder steerSensor;
    public double distance;
    public PIDController m_anglePID;
    public PIDController m_speedPID;
    public SwerveModuleState desiredState;

    public SwerveDriveWheel(TalonFX steerMotor, CANcoder steerSensor, TalonFX driveMotor)
    {
        this.driveMotor = driveMotor;
        this.steerSensor = steerSensor;
        this.steerMotor = steerMotor;
        distance = 0;
        m_anglePID = new PIDController(1.5, 0.0, 0.0);

        m_anglePID.enableContinuousInput(-0.5, 0.5);
        m_anglePID.setTolerance(0.01);

        Slot0Configs driveConfigs = new Slot0Configs();
        driveConfigs.kP = 0.05;
        driveConfigs.kV = 0.12;
        driveConfigs.kS = 0.10;

        Slot0Configs steerConfigs = new Slot0Configs(); //TUNE VALUES
        steerConfigs.kV = 0;
        steerConfigs.kP = 0.05;
        steerConfigs.kI = 0.01;
        steerConfigs.kD = 0.005;

        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 80;
        limitConfigs.StatorCurrentLimitEnable = true;

        driveMotor.getConfigurator().apply(driveConfigs);
        driveMotor.getConfigurator().apply(limitConfigs);
        steerMotor.getConfigurator().apply(steerConfigs);

        desiredState = new SwerveModuleState();

    }

    public void set(SwerveModuleState state)
    {
        double a = m_anglePID.calculate(state.angle.getRotations(), getAngle().getRotations());
        SmartDashboard.putNumber("MANUAL TARGET", a);
        if (m_anglePID.atSetpoint()){
            steerMotor.set(0);
        } else {
            steerMotor.set(a);
        }

        driveMotor.setControl(new VelocityVoltage(state.speedMetersPerSecond / Constants.Swerve.METERS_PER_ROT));

        distance += state.speedMetersPerSecond * 0.02;
        desiredState = state;
    }

    public double getPosition(){
        if (RobotBase.isReal()){
            return driveMotor.getRotorPosition().getValue() * Constants.Swerve.METERS_PER_ROT;
        } else {    
            return distance;
        }
    }

    public double getSpeed(){
        if (RobotBase.isReal()){
            return driveMotor.getRotorVelocity().getValue() * Constants.Swerve.METERS_PER_ROT;
        } else {
            return desiredState.speedMetersPerSecond;
        }
    }

    public Rotation2d getAngle(){
        if (RobotBase.isReal()){
            double v = steerSensor.getAbsolutePosition().getValue();
            return Rotation2d.fromRotations(v);
        } else {
            return desiredState.angle;
        }
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getPosition(), getAngle());
    }
    
}