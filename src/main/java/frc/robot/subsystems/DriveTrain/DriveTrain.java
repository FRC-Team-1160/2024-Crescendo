/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.DriveTrain;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder; 
//SWITCH TO PHOENIX6
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

import edu.wpi.first.wpilibj.RobotBase;


import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.DriverStation;

public class DriveTrain extends SubsystemBase {
  /** 
   * Creates a new DriveTrain.
   */

  private Joystick m_mainStick = new Joystick(0);
  
  private static DriveTrain m_instance;

  private TalonFX m_frontLeftSteerMotor, m_frontRightSteerMotor, m_backLeftSteerMotor, m_backRightSteerMotor;

  private Slot0Configs driveConfigs;

  private Slot0Configs steerConfigs;

  //private TalonFXSensorCollection m_frontLeftRotationEncoder, m_frontRightRotationEncoder, m_backLeftRotationEncoder, m_backRightRotationEncoder;

  private TalonFX m_frontLeftDriveMotor, m_frontRightDriveMotor, m_backLeftDriveMotor, m_backRightDriveMotor;

  //private TalonFXSensorCollection m_frontLeftDirectionEncoder, m_frontRightDirectionEncoder, m_backLeftDirectionEncoder, m_backRightDirectionEncoder;

  public SwerveDriveWheel m_frontLeftWheel, m_frontRightWheel, m_backLeftWheel, m_backRightWheel;
  
  public CANcoder m_frontLeftCoder, m_frontRightCoder, m_backLeftCoder, m_backRightCoder;

  private AHRS m_gyro;

  private Translation2d m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation;

  private SwerveDriveKinematics m_kinematics;

  public SwerveDrivePoseEstimator m_poseEstimator;

  private Solenoid m_gate;

  public SwerveModuleState[] m_moduleStates;

  public SwerveModulePosition[] m_modulePositions;

  public Pose2d odomPose;

  public Field2d m_field;

  public StructArrayPublisher<SwerveModuleState> adv_statesPub;

  public StructPublisher<Rotation2d> adv_gyroPub;

  public StructPublisher<Pose2d> adv_posePub;

  public double sim_angle;

  private PIDController m_anglePID;
  double wkP, wkI, wkD;

  public SlewRateLimiter zlimiter;

  //initializes the drive train
  
  public static DriveTrain getInstance(){
    if (m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  public DriveTrain() {

    zlimiter = new SlewRateLimiter(2);

    //directional motors

    m_frontLeftDriveMotor = new TalonFX(PortConstants.FRONT_LEFT_DRIVE_MOTOR);
    m_frontRightDriveMotor = new TalonFX(PortConstants.FRONT_RIGHT_DRIVE_MOTOR);
    m_backLeftDriveMotor = new TalonFX(PortConstants.BACK_LEFT_DRIVE_MOTOR);
    m_backRightDriveMotor = new TalonFX(PortConstants.BACK_RIGHT_DRIVE_MOTOR);

    //rotational motors
    m_frontLeftSteerMotor = new TalonFX(PortConstants.FRONT_LEFT_STEER_MOTOR);
    m_frontRightSteerMotor = new TalonFX(PortConstants.FRONT_RIGHT_STEER_MOTOR);
    m_backLeftSteerMotor = new TalonFX(PortConstants.BACK_LEFT_STEER_MOTOR);
    m_backRightSteerMotor = new TalonFX(PortConstants.BACK_RIGHT_STEER_MOTOR);

    //CAN coders
    m_frontLeftCoder = new CANcoder(PortConstants.FRONT_LEFT_CODER);
    m_frontRightCoder = new CANcoder(PortConstants.FRONT_RIGHT_CODER);
    m_backLeftCoder = new CANcoder(PortConstants.BACK_LEFT_CODER);
    m_backRightCoder = new CANcoder(PortConstants.BACK_RIGHT_CODER);

    // m_frontLeftCoder.setPosition(0);
    // m_frontRightCoder.setPosition(0);
    // m_backLeftCoder.setPosition(0);
    // m_backRightCoder.setPosition(0);

    //swerve wheel PID values
    // wkP = 0.005;
    // wkI = 0.000001;
    // wkD = 0;

    driveConfigs = new Slot0Configs(); //TUNE VALUES
    driveConfigs.kV = 2; //values taken from api ex for "basic use"
    driveConfigs.kP = 1;
    driveConfigs.kI = 0;
    driveConfigs.kD = 10;

    steerConfigs = new Slot0Configs(); //TUNE VALUES
    steerConfigs.kV = 0;
    steerConfigs.kP = 0.05;
    steerConfigs.kI = 0.01;
    steerConfigs.kD = 0.005;

    m_frontLeftDriveMotor.getConfigurator().apply(driveConfigs);
    m_frontRightDriveMotor.getConfigurator().apply(driveConfigs);
    m_backLeftDriveMotor.getConfigurator().apply(driveConfigs);
    m_backRightDriveMotor.getConfigurator().apply(driveConfigs);

    m_frontLeftSteerMotor.getConfigurator().apply(steerConfigs);
    m_frontRightSteerMotor.getConfigurator().apply(steerConfigs);
    m_backLeftSteerMotor.getConfigurator().apply(steerConfigs);
    m_backRightSteerMotor.getConfigurator().apply(steerConfigs);

    //swerve wheels (controls the rotation and direction motors)


    m_frontLeftWheel = new SwerveDriveWheel(m_frontLeftSteerMotor, m_frontLeftCoder, m_frontLeftDriveMotor);
    m_frontRightWheel = new SwerveDriveWheel(m_frontRightSteerMotor, m_frontRightCoder, m_frontRightDriveMotor);
    m_backLeftWheel = new SwerveDriveWheel(m_backLeftSteerMotor, m_backLeftCoder, m_backLeftDriveMotor);
    m_backRightWheel = new SwerveDriveWheel(m_backRightSteerMotor, m_backRightCoder, m_backRightDriveMotor);

    m_gyro = new AHRS(Port.kMXP);

    double offset = 23.75 * 0.0254;
    m_frontLeftLocation = new Translation2d(offset, offset); //CHANGE (when mech is done smh)
    m_frontRightLocation = new Translation2d(offset, -offset); //couldnt find what units translation2d uses, assume meters??
    m_backLeftLocation = new Translation2d(-offset, offset); //23.75 inches
    m_backRightLocation = new Translation2d(-offset, -offset);

    // Creating kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    m_gyro.zeroYaw();

    m_moduleStates = new SwerveModuleState[4];

    m_modulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(m_frontLeftWheel.getPosition() * Math.PI * 0.0254 * 4 / 6.75, Rotation2d.fromRotations(m_frontLeftWheel.getAngle())),
      new SwerveModulePosition(m_frontRightWheel.getPosition() * Math.PI * 0.0254 * 4 / 6.75, Rotation2d.fromRotations(m_frontLeftWheel.getAngle())),
      new SwerveModulePosition(m_backLeftWheel.getPosition() * Math.PI * 0.0254 * 4 / 6.75, Rotation2d.fromRotations(m_frontLeftWheel.getAngle())),
      new SwerveModulePosition(m_backRightWheel.getPosition() * Math.PI * 0.0254 * 4 / 6.75, Rotation2d.fromRotations(m_frontLeftWheel.getAngle()))
    };

    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(getGyroAngle()), m_modulePositions, new Pose2d(new Translation2d(1.5, 5.5), new Rotation2d()));
    
    m_field = new Field2d();

    if (RobotBase.isReal()){
      m_anglePID = new PIDController(0.2, 0, 0.05);
    } else {
      m_anglePID = new PIDController(0.2, 0 , 0.01);
    }
    m_anglePID.enableContinuousInput(0, Math.PI * 2);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_swerve = inst.getTable("adv_swerve");
    adv_statesPub = adv_swerve.getStructArrayTopic("States", SwerveModuleState.struct).publish();
    adv_gyroPub = adv_swerve.getStructTopic("Gyro", Rotation2d.struct).publish();
    adv_posePub = adv_swerve.getStructTopic("Pose", Pose2d.struct).publish();

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
    
        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeftWheel.getAngle() * 360, null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeftWheel.getSpeed() * Math.PI * 0.2, null);
    
        builder.addDoubleProperty("Front Right Angle", () -> m_frontRightWheel.getAngle() * 360, null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRightWheel.getSpeed() * Math.PI * 0.2, null);
    
        builder.addDoubleProperty("Back Left Angle", () -> m_backLeftWheel.getAngle() * 360, null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeftWheel.getSpeed() * Math.PI * 0.2, null);
    
        builder.addDoubleProperty("Back Right Angle", () -> m_backRightWheel.getAngle() * 360, null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRightWheel.getSpeed() * Math.PI * 0.2, null);
    
        builder.addDoubleProperty("Robot Angle", () -> getGyroAngle(), null);
      }
    });

    SmartDashboard.putData("Odom Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
    
        builder.addDoubleProperty("Front Left Angle", () -> m_modulePositions[0].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeftWheel.getSpeed() * Math.PI * 0.2 * 4, null);
    
        builder.addDoubleProperty("Front Right Angle", () -> m_modulePositions[1].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRightWheel.getSpeed() * Math.PI * 0.2 * 4, null);
    
        builder.addDoubleProperty("Back Left Angle", () -> m_modulePositions[2].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeftWheel.getSpeed() * Math.PI * 0.2 * 4, null);
    
        builder.addDoubleProperty("Back Right Angle", () -> m_modulePositions[3].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRightWheel.getSpeed() * Math.PI * 0.2 * 4, null);
    
        builder.addDoubleProperty("Robot Angle", () -> getGyroAngle(), null);
      }
    });
    
    sim_angle = 0;
  }

  public double getGyroAngle() {
    if (RobotBase.isReal()){
      return -m_gyro.getYaw(); //GYRO REPORTS CW POSITIVE
    } else {
      if (Math.abs(sim_angle) > 180){
        sim_angle -= Math.signum(sim_angle) * 360;
      }
      m_gyro.setAngleAdjustment(sim_angle);
      return sim_angle;
    }
  }

  public void resetGyro() {
    m_gyro.zeroYaw();
    m_gyro.reset();
  }

  public void setGate(boolean b) {
    m_gate.set(b);
  }

  public boolean getGateStatus() {
    return m_gate.get();
  }


  //Thanks to Team 4738 for modified discretize code
  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose = new Pose2d(
      speeds.vxMetersPerSecond * dt, 
      speeds.vyMetersPerSecond * dt, 
      new Rotation2d(speeds.omegaRadiansPerSecond * dt * 1)
    );
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  /**
   * @param xSpeed
   * @param ySpeed
   * @param angSpeed
   */
  public void setSwerveDrive(double xSpeed, double ySpeed, double angSpeed){
    sim_angle += angSpeed * 20 * 0.0254 * 4 / 23.75 * 360;
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    //SmartDashboard.putNumber("INA", angSpeed);
    ChassisSpeeds m_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angSpeed, Rotation2d.fromDegrees(getGyroAngle()));
    //SmartDashboard.putNumber("OUTA", m_speeds.omegaRadiansPerSecond);
    m_speeds = discretize(m_speeds);
    SwerveModuleState[] m_moduleStates = (m_kinematics.toSwerveModuleStates(m_speeds));

    SmartDashboard.putNumber("m0", m_moduleStates[0].speedMetersPerSecond);

    m_moduleStates[0] = SwerveModuleState.optimize(m_moduleStates[0], Rotation2d.fromRotations(m_frontLeftWheel.getAngle()));
    m_moduleStates[1] = SwerveModuleState.optimize(m_moduleStates[1], Rotation2d.fromRotations(m_frontRightWheel.getAngle()));
    m_moduleStates[2] = SwerveModuleState.optimize(m_moduleStates[2], Rotation2d.fromRotations(m_backLeftWheel.getAngle()));
    m_moduleStates[3] = SwerveModuleState.optimize(m_moduleStates[3], Rotation2d.fromRotations(m_backRightWheel.getAngle()));

    SwerveDriveKinematics.desaturateWheelSpeeds(m_moduleStates, 5);

    adv_statesPub.set(m_moduleStates);

    //check the angle thing is right

    //meters per second is a lie (pretend its scaled 0 to 1)

    //SCALED 0 TO 1 IS A LIE (its actually rots/sec) (i think)
    
    m_frontLeftWheel.set(m_moduleStates[0].angle.getRotations(), m_moduleStates[0].speedMetersPerSecond);
    m_frontRightWheel.set(m_moduleStates[1].angle.getRotations(), m_moduleStates[1].speedMetersPerSecond);
    m_backLeftWheel.set(m_moduleStates[2].angle.getRotations(), m_moduleStates[2].speedMetersPerSecond);
    m_backRightWheel.set(m_moduleStates[3].angle.getRotations(), m_moduleStates[3].speedMetersPerSecond);

    m_modulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(m_frontLeftWheel.getPosition() * Math.PI * 4 * 0.0254 / 6.75, Rotation2d.fromRotations(m_frontLeftWheel.getAngle())),
      new SwerveModulePosition(m_frontRightWheel.getPosition() * Math.PI * 4 * 0.0254 / 6.75, Rotation2d.fromRotations(m_frontRightWheel.getAngle())),
      new SwerveModulePosition(m_backLeftWheel.getPosition() * Math.PI * 4 * 0.0254 / 6.75, Rotation2d.fromRotations(m_backLeftWheel.getAngle())),
      new SwerveModulePosition(m_backRightWheel.getPosition() * Math.PI * 4 * 0.0254 / 6.75, Rotation2d.fromRotations(m_backRightWheel.getAngle()))
    };


    // m_modulePositions = new SwerveModulePosition[] {
    //   new SwerveModulePosition(m_frontLeftWheel.getPosition() * Math.PI * 4 * 0.0254 / 6.75, new Rotation2d()),
    //   new SwerveModulePosition(m_frontRightWheel.getPosition() * Math.PI * 4 * 0.0254 / 6.75, new Rotation2d()),
    //   new SwerveModulePosition(m_backLeftWheel.getPosition() * Math.PI * 4 * 0.0254 / 6.75, new Rotation2d()),
    //   new SwerveModulePosition(m_backRightWheel.getPosition() * Math.PI * 4 * 0.0254 / 6.75, new Rotation2d())
    // };

    odomPose = m_poseEstimator.update(Rotation2d.fromDegrees(getGyroAngle()), m_modulePositions);
    SmartDashboard.putData("Gyro", m_gyro);

    m_field.setRobotPose(odomPose);

    adv_gyroPub.set(new Rotation2d(getGyroAngle()));
    adv_posePub.set(odomPose);
    SmartDashboard.putData("Field", m_field);

    // SmartDashboard.putData("Swerve Drive", new Sendable() {
    //   @Override
    //   public void initSendable(SendableBuilder builder) {
    //     builder.setSmartDashboardType("SwerveDrive");
    
    //     builder.addDoubleProperty("Front Left Angle", () -> m_frontLeftWheel.getAngle() * 360, null);
    //     builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeftWheel.getSpeed() * Math.PI * 0.2, null);
    
    //     builder.addDoubleProperty("Front Right Angle", () -> m_frontRightWheel.getAngle() * 360, null);
    //     builder.addDoubleProperty("Front Right Velocity", () -> m_frontRightWheel.getSpeed() * Math.PI * 0.2, null);
    
    //     builder.addDoubleProperty("Back Left Angle", () -> m_backRightWheel.getAngle() * 360, null);
    //     builder.addDoubleProperty("Back Left Velocity", () -> m_backRightWheel.getSpeed() * Math.PI * 0.2, null);
    
    //     builder.addDoubleProperty("Back Right Angle", () -> m_backRightWheel.getAngle() * 360, null);
    //     builder.addDoubleProperty("Back Right Velocity", () -> m_backRightWheel.getSpeed() * Math.PI * 0.2, null);
    
    //     builder.addDoubleProperty("Robot Angle", () -> getGyroAngle() * 360, null);
    //   }
    // });

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    SmartDashboard.putNumber("FLMotorAngle", m_frontLeftWheel.getAngle());
    SmartDashboard.putNumber("FLCoderAngle", m_frontLeftCoder.getPosition().getValue());
    SmartDashboard.putNumber("FLTargetAngle", m_moduleStates[0].angle.getRotations());
    SmartDashboard.putNumber("FLAngle", m_frontLeftWheel.getAngle());
    SmartDashboard.putNumber("FLTarget", m_moduleStates[0].speedMetersPerSecond);

    SmartDashboard.putNumber("FRMotorAngle", m_frontRightWheel.getAngle());
    SmartDashboard.putNumber("FRCoderAngle", m_frontRightCoder.getPosition().getValue());
    SmartDashboard.putNumber("FRTargetAngle", m_moduleStates[1].angle.getRotations());
    SmartDashboard.putNumber("FRAngle", m_frontRightWheel.getAngle());
    SmartDashboard.putNumber("FRTarget", m_moduleStates[1].speedMetersPerSecond);

    SmartDashboard.putNumber("BLMotorAngle", m_backLeftWheel.getAngle());
    SmartDashboard.putNumber("BLCoderAngle", m_backLeftCoder.getPosition().getValue());
    SmartDashboard.putNumber("BLTargetAngle", m_moduleStates[2].angle.getRotations());
    SmartDashboard.putNumber("BLAngle", m_backLeftWheel.getAngle());
    SmartDashboard.putNumber("BLTarget", m_moduleStates[2].speedMetersPerSecond);

    SmartDashboard.putNumber("BRMotorAngle", m_backRightWheel.getAngle());
    SmartDashboard.putNumber("BRCoderAngle", m_backRightCoder.getPosition().getValue());
    SmartDashboard.putNumber("BRTargetAngle", m_moduleStates[3].angle.getRotations());
    SmartDashboard.putNumber("BRAngle", m_backRightWheel.getAngle());
    SmartDashboard.putNumber("BRTarget", m_moduleStates[3].speedMetersPerSecond);


  }

  public double aimSwerveDrive(double xSpeed, double ySpeed, double targetX, double targetY){
    double target = Math.atan2(targetY - odomPose.getY(), targetX - odomPose.getX());
    //double diff = getGyroAngle() * Math.PI / 180.0 - angle;
    double angle = getGyroAngle() * Math.PI / 180.0;
    if (target > Math.PI * 2){
      target -= Math.PI * 2;
    }
    if (angle < 0){
      angle += Math.PI * 2;
    }
    System.out.print(angle + "\t\t");
    System.out.println(target);
    double max = 0.25;
    double angSpeed = Math.max(Math.min(m_anglePID.calculate(angle, target), max), -max);
    if (Math.abs(angSpeed) < 0.01){
      angSpeed = 0;
    }
    setSwerveDrive(xSpeed, ySpeed, angSpeed);
    return target;
  }

  public double[] inputSpeeds(){
    // double joystickX = m_mainStick.getRawAxis(1);
    // double joystickY = m_mainStick.getRawAxis(0);
    // double joystickA = m_mainStick.getRawAxis(4);
    double joystickX = -m_mainStick.getRawAxis(1);
    double joystickY = -m_mainStick.getRawAxis(0);
    double joystickA = -m_mainStick.getRawAxis(2);

    double x = 0.0;
    double y = 0.0;
    double a = 0.0;

    if (Math.abs(joystickX) > 0.1){
      x = joystickX;
    }
    if (Math.abs(joystickY) > 0.1){
      y = joystickY;
    }
    if (Math.abs(joystickA) > 0.15){
      a = joystickA;
    }

    a = zlimiter.calculate(a);
    SmartDashboard.putNumber("zinputlimited", a);

    double dir = Math.atan2(y, x);
    double mag = Math.sqrt(x*x + y*y);
    if (mag > 1.0){
      mag = 1.0;
    }
    x = Math.cos(dir) * 1 * Math.pow(mag, 2);
    y = Math.sin(dir) * 1 * Math.pow(mag, 2);
    a = Math.signum(a) * 0.5 * (Math.pow(a, 2) / (1 + Math.sqrt(mag) / 2));

    SmartDashboard.putNumber("inX", x);
    SmartDashboard.putNumber("inY", y);
    SmartDashboard.putNumber("inA", a);
    
    return new double[]{x, y, a};

  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute", m_frontLeftCoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("regular", m_frontLeftCoder.getPosition().getValue());

  }
}