package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

  public boolean revved;
  public boolean aimed;

  public double rpm_speed;

  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }

  private Shooter(){
    topMotor = new CANSparkMax(Port.SHOOTER_TOP_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(Port.SHOOTER_BOTTOM_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    topPID = topMotor.getPIDController();
    bottomPID = bottomMotor.getPIDController();

    topPID.setP(0.0002);
    topPID.setFF(0.000177);
    bottomPID.setP(0.0002);
    bottomPID.setFF(0.000177);

    pitchMotor = new CANSparkMax(Port.SHOOTER_PITCH_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    pitchPID = new PIDController(3.5, 20.0, 0);
    pitchPID.setIntegratorRange(-0.006, 0.006);
    // pitchMotor.getAlternateEncoder(8192).setPosition(0);
    setpoint = pitchMotor.getAlternateEncoder(8192).getPosition();
    manual = 0;
    SmartDashboard.putNumber("Shooter Pitch", 0);
    SmartDashboard.putBoolean("Shooter Aimed", false);
    SmartDashboard.putBoolean("Shooter Revved", false);
    m_drive = DriveTrain.getInstance();
    setSpeed(0);
    revved = false;
    aimed = false;
  }

  public void setSpeed(double speed){
    speed = Math.max(-1, Math.min(1, speed));
    if (speed == 0){
      topPID.setReference(0, ControlType.kVoltage);
      bottomPID.setReference(0, ControlType.kVoltage);
    } else {
      topPID.setReference(speed * 5500, ControlType.kVelocity);
      bottomPID.setReference(-speed * 5500, ControlType.kVelocity);
    }
    SmartDashboard.putNumber("Shooter Speed", speed);
  }

  public double aimTarget(double x, double y, double z){
    double dist = Math.sqrt(Math.pow(m_drive.odomPose.getX() - x, 2) + Math.pow(m_drive.odomPose.getY() - y, 2));
    double angle = Math.min(0.17, Math.atan2(z, dist) * 0.107 / (Math.PI / 4)) - 0.001;
    setpoint = angle;
    SmartDashboard.putNumber("Shooter Pitch", angle);
    return angle;
  }

  public double revTarget(double x, double y){
    // double dist = Math.sqrt(Math.pow(m_drive.odomPose.getX() - x, 2) + Math.pow(m_drive.odomPose.getY() - y, 2));
    // double s = Math.min(0.5 + dist/5.0, 1.0);
    // SmartDashboard.putNumber("Shooter Speed", s);
    // SmartDashboard.putBoolean("Shooter Revved", (topMotor.getEncoder().getVelocity() > 4000 * s));
    // setSpeed(s);
    // return s;
    double dist = Math.sqrt(Math.pow(m_drive.odomPose.getX() - x, 2) + Math.pow(m_drive.odomPose.getY() - y, 2));
    double rpm_speed = Math.min(0.6 + dist/5.0, 1.0);
    SmartDashboard.putNumber("Shooter Speed", rpm_speed);
    SmartDashboard.putBoolean("Shooter Revved", revved);
    setSpeed(rpm_speed);
    return rpm_speed;
  }

  public void periodic(){
    revved = (topMotor.getEncoder().getVelocity() > 5000 * rpm_speed) && topMotor.getEncoder().getVelocity() > 100;
    speed = SmartDashboard.getNumber("Shooter Speed", 0.5);
    SmartDashboard.putNumber("Shooter RPM", topMotor.getEncoder().getVelocity());
    setSpeed(speed);
    double position = pitchMotor.getAlternateEncoder(8192).getPosition();
    position = Math.max(position, 0);
    SmartDashboard.putNumber("Pitch Encoder", position);
    setpoint = SmartDashboard.getNumber("Shooter Pitch", setpoint);
    SmartDashboard.putNumber("Shooter Pitch", setpoint);
    double v = Math.max(-0.15, Math.min(0.15, pitchPID.calculate(position, setpoint)));
    SmartDashboard.putNumber("PitchPID", v);
    v += 0.092 * Math.sqrt(position);
    SmartDashboard.putNumber("PIDwithFF", v);
    manual = Math.min(0.2, Math.abs(SmartDashboard.getNumber("ManualWrist", 0)));
    pitchMotor.set(-v);

    SmartDashboard.putNumber("ManualWrist", manual);

    aimed = Math.abs(setpoint - position) < 0.008;
    SmartDashboard.putBoolean("Aimed", aimed);

    
  }

}
