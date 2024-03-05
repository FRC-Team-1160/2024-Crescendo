package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PortConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class Shooter extends SubsystemBase{

  private static Shooter m_instance;
  
  public CANSparkMax topMotor;
  public CANSparkMax bottomMotor;
  public CANSparkMax pitchMotor;
  public PIDController pitchPID;
  
  public double speed;
  public double setpoint;

  public DriveTrain m_drive;

  public double manual;

  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }

  private Shooter(){
    topMotor = new CANSparkMax(PortConstants.SHOOTER_TOP_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(PortConstants.SHOOTER_BOTTOM_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    pitchMotor = new CANSparkMax(PortConstants.SHOOTER_PITCH_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    pitchPID = new PIDController(3.5, 0, 0.05);

    // pitchMotor.getAlternateEncoder(8192).setPosition(0);
    setpoint = pitchMotor.getAlternateEncoder(8192).getPosition();
    speed = 0;
    manual = 0;
    SmartDashboard.putNumber("Shooter Pitch", 0);
    m_drive = DriveTrain.getInstance();
  }

  public void setSpeed(double speed){
    this.speed = speed;
    topMotor.set(speed);
    bottomMotor.set(-speed);
    SmartDashboard.putNumber("Shooter Speed", speed);
  }

  public double aimSpeaker(){
    double dist = Math.sqrt(Math.pow(m_drive.odomPose.getX() - 0.5, 2) + Math.pow(m_drive.odomPose.getY() - 5.5, 2));
    double angle = Math.atan2(0.38, dist) * 0.107 / (Math.PI / 4);
    //setpoint = angle
    return angle;
  }

  public double revSpeaker(){
    double dist = Math.sqrt(Math.pow(m_drive.odomPose.getX() - 0.5, 2) + Math.pow(m_drive.odomPose.getY() - 5.5, 2));
    double s = Math.min(0.5 + dist/10.0, 1.0);
    //speed = s
    return s;
  }

  public void periodic(){
    speed = SmartDashboard.getNumber("Shooter Speed", 0);
    setSpeed(speed);
    double position = pitchMotor.getAlternateEncoder(8192).getPosition();
    position = Math.max(position, 0);
    SmartDashboard.putNumber("Pitch Encoder", position);
    setpoint = SmartDashboard.getNumber("Shooter Pitch", setpoint);
    SmartDashboard.putNumber("Shooter Pitch", setpoint);
    double v = pitchPID.calculate(position, setpoint);
    v += 0.1 * Math.sqrt(position);

    SmartDashboard.putNumber("pitchPID", v);
    manual = Math.min(0.2, Math.abs(SmartDashboard.getNumber("ManualWrist", 0)));
    pitchMotor.set(-v);

    SmartDashboard.putNumber("ManualWrist", manual);
  }

}
