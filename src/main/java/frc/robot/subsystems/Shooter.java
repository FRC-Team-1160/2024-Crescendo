package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Port;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class Shooter extends SubsystemBase{

  private static Shooter m_instance;
  
  public CANSparkMax topMotor;
  public CANSparkMax bottomMotor;
  public SparkPIDController topPID;
  public SparkPIDController bottomPID;

  public CANSparkMax pitchMotor;
  public PIDController pitchPID;
  
  public double setpoint;

  public DriveTrain m_drive;

  public double speed;

  public double manual;

  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }

  private Shooter(){
    topMotor = new CANSparkMax(Port.SHOOTER_TOP_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(Port.SHOOTER_BOTTOM_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    pitchMotor = new CANSparkMax(Port.SHOOTER_PITCH_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    pitchPID = new PIDController(0.1, 0.0, 0.00);

    speed = 0;

    topPID = topMotor.getPIDController();
    bottomPID = bottomMotor.getPIDController();

    topPID.setP(0.0002);
    topPID.setFF(0.000177);
    bottomPID.setP(0.0002);
    bottomPID.setFF(0.000177);

    // pitchMotor.getAlternateEncoder(8192).setPosition(0);
    setpoint = pitchMotor.getAlternateEncoder(8192).getPosition();
    manual = 0;
    SmartDashboard.putNumber("Shooter Pitch", 0);
    SmartDashboard.putBoolean("Shooter Aimed", false);
    SmartDashboard.putBoolean("Shooter Revved", false);
    m_drive = DriveTrain.getInstance();
  }

  public void setSpeed(double speed){
    speed = Math.max(-1, Math.min(1, speed));
    topPID.setReference(speed * 5500, ControlType.kVelocity);
    bottomPID.setReference(speed * 5500, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter Speed", speed);
  }

  public double aimTarget(double x, double y, double z){
    double dist = Math.sqrt(Math.pow(m_drive.odomPose.getX() - x, 2) + Math.pow(m_drive.odomPose.getY() - y, 2));
    double angle = Math.min(0.16, Math.atan2(z, dist) * 0.11 / (Math.PI / 4));
    setpoint = angle;
    SmartDashboard.putNumber("Shooter Pitch", angle);
    SmartDashboard.putBoolean("Shooter Aimed", (Math.abs(setpoint - angle) < 0.005));
    return angle;
  }

  public double revTarget(double x, double y){
    double dist = Math.sqrt(Math.pow(m_drive.odomPose.getX() - x, 2) + Math.pow(m_drive.odomPose.getY() - y, 2));
    double s = Math.min(0.4 + dist/6.0, 1.0);
    SmartDashboard.putNumber("Shooter Speed", s);
    SmartDashboard.putBoolean("Shooter Revved", (topMotor.getEncoder().getVelocity() > 4000 * s));
    setSpeed(s);
    return s;
  }

  public void presetAmp(){
    setpoint = 16.5;
    setSpeed(0.27);
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", topMotor.getEncoder().getVelocity());
    setSpeed(SmartDashboard.getNumber("Shooter Speed", 0));
    double position = pitchMotor.getAlternateEncoder(8192).getPosition();
    position = Math.max(position, 0);
    SmartDashboard.putNumber("Pitch Encoder", position);
    setpoint = SmartDashboard.getNumber("Shooter Pitch", setpoint);
    SmartDashboard.putNumber("Shooter Pitch", setpoint);
    double v = Math.max(-0.2, Math.min(0.2, pitchPID.calculate(position, setpoint)));
    SmartDashboard.putNumber("PitchPID", v);
    manual = Math.min(0.2, Math.abs(SmartDashboard.getNumber("ManualWrist", 0)));
    pitchMotor.set(-v);

    SmartDashboard.putNumber("ManualWrist", manual);
    
  }

}
