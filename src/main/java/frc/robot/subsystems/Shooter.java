package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PortConstants;

public class Shooter extends SubsystemBase{

  private static Shooter m_instance;
  
  public CANSparkMax topMotor;
  public CANSparkMax bottomMotor;
  public CANSparkMax pitchMotor;
  public PIDController pitchPID;
  
  public double speed;
  public double setpoint;

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
    pitchPID = new PIDController(3.5, 0, 0.05); //i used to be 0.8, IZone 0.06
    //pulling a marcus because variable feedforward

    // pitchMotor.getAlternateEncoder(8192).setPosition(0);
    setpoint = pitchMotor.getAlternateEncoder(8192).getPosition();
    speed = 0;
    manual = 0;
    SmartDashboard.putNumber("Shooter Pitch", 0);
  }

  public void setSpeed(double speed){
    this.speed = speed;
    topMotor.set(speed);
    bottomMotor.set(-speed);
    SmartDashboard.putNumber("Shooter Speed", speed);
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
    // if (Math.abs(position - setpoint) >= 0.005){
    //   v += 0.02 * Math.signum(v);
    //}
    SmartDashboard.putNumber("pitchPID", v);
    manual = Math.min(0.2, Math.abs(SmartDashboard.getNumber("ManualWrist", 0)));
    pitchMotor.set(-v);

    SmartDashboard.putNumber("ManualWrist", manual);
  }

}
