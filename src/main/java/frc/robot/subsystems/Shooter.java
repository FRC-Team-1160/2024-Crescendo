package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.Port;
import frc.robot.subsystems.DriveTrain.DriveTrain;

import java.lang.reflect.*;  


public class Shooter extends SubsystemBase{

  private static Shooter m_instance;
  
  // public CANSparkMax topMotor;
  // public CANSparkMax bottomMotor;
  public TalonFX topMotor;
  public TalonFX bottomMotor;
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

  public Joystick m_rightBoard;

  public double offset;

  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  public Color m_color;
  public String ledState;

  public Spark blinkin;
  public PWM m_pwm;


  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }

  private Shooter(){

    // topMotor = new CANSparkMax(Port.SHOOTER_TOP_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    // bottomMotor = new CANSparkMax(Port.SHOOTER_BOTTOM_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    // topPID = topMotor.getPIDController();
    // bottomPID = bottomMotor.getPIDController();

    topMotor = new TalonFX(Port.SHOOTER_TOP_MOTOR);
    bottomMotor = new TalonFX(Port.SHOOTER_BOTTOM_MOTOR);

    // topPID.setP(0.0002);
    // topPID.setFF(0.000177);
    // bottomPID.setP(0.0002);
    // bottomPID.setFF(0.000177);
    var configs = new Slot0Configs();

    configs.kP = 0.1;
    configs.kV = 0.116;
    configs.kI = 0.0;
    configs.kD = 0.0;
    configs.kS = 0.0;
    configs.kA = 0.0;
    configs.kG = 0.0;

    topMotor.getConfigurator().apply(configs);
    bottomMotor.getConfigurator().apply(configs);

    topMotor.setNeutralMode(NeutralModeValue.Coast);
    bottomMotor.setNeutralMode(NeutralModeValue.Coast);

    pitchMotor = new CANSparkMax(Port.SHOOTER_PITCH_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    pitchPID = new PIDController(3.5, 50.0, 0);
    pitchPID.setIntegratorRange(-0.005, 0.005);
    pitchPID.setIZone(0.02);
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
    offset = 0;
    speed = 0;
    m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);
    SmartDashboard.putNumber("TopMult", 1);
    SmartDashboard.putNumber("BottomMult", 1);

    blinkin = new Spark(0);
    blinkin.set(0.93);
    m_pwm = new PWM(0);

  }

  public void setSpeed(double s) {
    speed = Math.max(-1, Math.min(1, s)) * 50;
    if (speed == 0){
      // topPID.setReference(0, ControlType.kVoltage);
      // bottomPID.setReference(0, ControlType.kVoltage);
      topMotor.setControl(new CoastOut());
      bottomMotor.setControl(new CoastOut());
    } else {
      // topPID.setReference(speed * 5500, ControlType.kVelocity);
      // bottomPID.setReference(-speed * 5500, ControlType.kVelocity);
      // bottomMotor.set(s);
      // topMotor.set(s);
      SmartDashboard.putNumber("ShooterTarget", speed * 60);
      topMotor.setControl(new VelocityVoltage(speed * SmartDashboard.getNumber("TopMult", 1)).withSlot(0));
      bottomMotor.setControl(new VelocityVoltage(speed * SmartDashboard.getNumber("BottomMult", 1)).withSlot(0));
    }
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
    double rpm_speed = Math.min(0.6 + dist/8.0, 1.0);
    setSpeed(rpm_speed);
    return rpm_speed;
  }


  public void periodic(){

    double t_rpm = topMotor.getRotorVelocity().getValue() * 60;
    double b_rpm = bottomMotor.getRotorVelocity().getValue() * 60;
    revved = Math.min(t_rpm, b_rpm) > speed * 60 - 200 && Math.min(t_rpm, b_rpm) > 100 && speed > 0;
    SmartDashboard.putBoolean("Shooter Revved", revved);
    SmartDashboard.putNumber("Shooter RPM Top", topMotor.getRotorVelocity().getValue() * 60);
    SmartDashboard.putNumber("Shooter RPM Bottom", bottomMotor.getRotorVelocity().getValue() * 60);
    SmartDashboard.putNumber("Shooter Offset", offset);
    SmartDashboard.putNumber("Shooter Speed", speed * 60);
    double position = pitchMotor.getAlternateEncoder(8192).getPosition();
    position = Math.max(position, 0);
    SmartDashboard.putNumber("Pitch Encoder", position);
    double v = Math.max(-0.25, Math.min(0.2, pitchPID.calculate(position, setpoint)));
    SmartDashboard.putNumber("PitchPID", v);
    v += 0.10 * Math.sqrt(position);
    SmartDashboard.putNumber("PitchPIDwFF", v);
    manual = SmartDashboard.getNumber("ManualWrist", manual);

    pitchMotor.set(-v);

    SmartDashboard.putNumber("ManualWrist", manual);

    aimed = Math.abs(setpoint - position) < 0.008;
    SmartDashboard.putBoolean("Shooter Aimed", aimed);
  }

}
