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
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  private Joystick m_secondStick = new Joystick(1);
  
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
  public SwerveDriveKinematics m_kinematics;
  public SwerveDrivePoseEstimator m_poseEstimator;
  public SwerveModuleState[] m_moduleStates;
  public SwerveModulePosition[] m_modulePositions;
  public Pose2d odomPose;
  public Field2d m_field;
  public StructArrayPublisher<SwerveModuleState> adv_statesPub;
  public StructArrayPublisher<SwerveModuleState> adv_targetStatesPub;
  public StructPublisher<Rotation2d> adv_gyroPub;
  public StructPublisher<Pose2d> adv_posePub;
  public double sim_angle;
  public PIDController m_anglePID;
  double wkP, wkI, wkD;
  public SlewRateLimiter zlimiter;
  //initializes the drive train
  public int multiplier;
  public boolean isRed;
  public boolean aimed;
  
  public static DriveTrain getInstance(){
    if (m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  public DriveTrain() {
    AutoBuilder.configureHolonomic(
      () -> odomPose,
      (Pose2d pose) -> odomPose = pose,
      () -> m_kinematics.toChassisSpeeds(m_moduleStates),
      this::setSwerveDrive,
      new HolonomicPathFollowerConfig(
        new PIDConstants(0.1),
        new PIDConstants(0, 0.0, 0.00),
        1,
        Constants.Swerve.BASE_RADIUS,
        new ReplanningConfig()
      ),
      () -> false,
      this
    );

    zlimiter = new SlewRateLimiter(5);

    //directional motors

    m_frontLeftDriveMotor = new TalonFX(Constants.Port.FRONT_LEFT_DRIVE_MOTOR, "*");
    m_frontRightDriveMotor = new TalonFX(Constants.Port.FRONT_RIGHT_DRIVE_MOTOR, "*");
    m_backLeftDriveMotor = new TalonFX(Constants.Port.BACK_LEFT_DRIVE_MOTOR, "*");
    m_backRightDriveMotor = new TalonFX(Constants.Port.BACK_RIGHT_DRIVE_MOTOR, "*");

    //rotational motors
    m_frontLeftSteerMotor = new TalonFX(Constants.Port.FRONT_LEFT_STEER_MOTOR, "*");
    m_frontRightSteerMotor = new TalonFX(Constants.Port.FRONT_RIGHT_STEER_MOTOR, "*");
    m_backLeftSteerMotor = new TalonFX(Constants.Port.BACK_LEFT_STEER_MOTOR, "*");
    m_backRightSteerMotor = new TalonFX(Constants.Port.BACK_RIGHT_STEER_MOTOR, "*");

    //CAN coders
    m_frontLeftCoder = new CANcoder(Constants.Port.FRONT_LEFT_CODER, "*");
    m_frontRightCoder = new CANcoder(Constants.Port.FRONT_RIGHT_CODER, "*");
    m_backLeftCoder = new CANcoder(Constants.Port.BACK_LEFT_CODER, "*");
    m_backRightCoder = new CANcoder(Constants.Port.BACK_RIGHT_CODER, "*");

    // m_frontLeftCoder.setPosition(0);
    // m_frontRightCoder.setPosition(0);
    // m_backLeftCoder.setPosition(0);
    // m_backRightCoder.setPosition(0);

    //swerve wheels (controls the rotation and direction motors)

    m_frontLeftWheel = new SwerveDriveWheel(m_frontLeftSteerMotor, m_frontLeftCoder, m_frontLeftDriveMotor);
    m_frontRightWheel = new SwerveDriveWheel(m_frontRightSteerMotor, m_frontRightCoder, m_frontRightDriveMotor);
    m_backLeftWheel = new SwerveDriveWheel(m_backLeftSteerMotor, m_backLeftCoder, m_backLeftDriveMotor);
    m_backRightWheel = new SwerveDriveWheel(m_backRightSteerMotor, m_backRightCoder, m_backRightDriveMotor);

    m_gyro = new AHRS(Port.kMXP);
    sim_angle = 0;

    double offset = Constants.Swerve.SIDE_LENGTH/2;
    m_frontLeftLocation = new Translation2d(offset, offset);
    m_frontRightLocation = new Translation2d(offset, -offset); 
    m_backLeftLocation = new Translation2d(-offset, offset); 
    m_backRightLocation = new Translation2d(-offset, -offset);

    // Creating kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    m_gyro.zeroYaw();

    m_moduleStates = new SwerveModuleState[]{
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
    };

    m_modulePositions = new SwerveModulePosition[] {
      m_frontLeftWheel.getModulePosition(),
      m_frontRightWheel.getModulePosition(),
      m_backLeftWheel.getModulePosition(),
      m_backRightWheel.getModulePosition()
    };

    odomPose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(getGyroAngle()), m_modulePositions, odomPose);
    
    m_field = new Field2d();

    if (RobotBase.isReal()){
      m_anglePID = new PIDController(5.0, 0, 0.1);
    } else {
      m_anglePID = new PIDController(5.0, 0 , 0.1);
    }
    m_anglePID.enableContinuousInput(-Math.PI, Math.PI);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_swerve = inst.getTable("adv_swerve");
    adv_statesPub = adv_swerve.getStructArrayTopic("States", SwerveModuleState.struct).publish();
    adv_targetStatesPub = adv_swerve.getStructArrayTopic("Target States", SwerveModuleState.struct).publish();
    adv_gyroPub = adv_swerve.getStructTopic("Gyro", Rotation2d.struct).publish();
    adv_posePub = adv_swerve.getStructTopic("Pose", Pose2d.struct).publish();

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
    
        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeftWheel.getAngle().getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeftWheel.getSpeed(), null);
    
        builder.addDoubleProperty("Front Right Angle", () -> m_frontRightWheel.getAngle().getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRightWheel.getSpeed(), null);
    
        builder.addDoubleProperty("Back Left Angle", () -> m_backLeftWheel.getAngle().getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeftWheel.getSpeed(), null);
    
        builder.addDoubleProperty("Back Right Angle", () -> m_backRightWheel.getAngle().getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRightWheel.getSpeed(), null);
    
        builder.addDoubleProperty("Robot Angle", () -> getGyroAngle() * Math.PI / 180, null);
      }
    });

    SmartDashboard.putData("Swerve States", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
    
        builder.addDoubleProperty("Front Left Angle", () -> m_moduleStates[0].angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_moduleStates[0].speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Front Right Angle", () -> m_moduleStates[1].angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_moduleStates[1].speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Back Left Angle", () -> m_moduleStates[2].angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_moduleStates[2].speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Back Right Angle", () -> m_moduleStates[3].angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_moduleStates[3].speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Robot Angle", () -> getGyroAngle() * Math.PI / 180, null);
      }
    });
    
    isRed = DriverStation.getAlliance().get() == Alliance.Red;

  }

  public double getGyroAngle() {
    if (RobotBase.isReal()){
      return -MathUtil.inputModulus(m_gyro.getAngle(), -180, 180); //GYRO REPORTS CW POSITIVE, RETURN CCW POSITIVE
    } else {
      return MathUtil.inputModulus(sim_angle, -180, 180);
    }
  }

  public void resetGyro() {
    m_gyro.zeroYaw();
    m_gyro.reset();
    sim_angle = 0;
  }

  public void resetPose(Pose2d n_pose) {
    m_poseEstimator.resetPosition(
      n_pose.getRotation(),
      m_modulePositions,
      n_pose);
    odomPose = n_pose;
    m_gyro.zeroYaw();
    m_gyro.reset();
    double deg = n_pose.getRotation().getDegrees();
    if (deg > 180){
      deg -= 360;
    }
    m_gyro.setAngleAdjustment(deg);
    sim_angle = -deg;
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
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    ChassisSpeeds m_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angSpeed, Rotation2d.fromDegrees(getGyroAngle()));
    setSwerveDrive(m_speeds);
  }

  public void setSwerveDrive(ChassisSpeeds speeds) {
    SmartDashboard.putString("chassis", speeds.toString());
    sim_angle += speeds.omegaRadiansPerSecond * 0.02 * 180 / Math.PI;
    speeds = discretize(speeds);
    SwerveModuleState[] m_moduleStates = (m_kinematics.toSwerveModuleStates(speeds));

    SmartDashboard.putNumber("m0", m_moduleStates[0].speedMetersPerSecond);

    m_moduleStates[0] = SwerveModuleState.optimize(m_moduleStates[0], m_frontLeftWheel.getAngle());
    m_moduleStates[1] = SwerveModuleState.optimize(m_moduleStates[1], m_frontRightWheel.getAngle());
    m_moduleStates[2] = SwerveModuleState.optimize(m_moduleStates[2], m_backLeftWheel.getAngle());
    m_moduleStates[3] = SwerveModuleState.optimize(m_moduleStates[3], m_backRightWheel.getAngle());

    SwerveDriveKinematics.desaturateWheelSpeeds(m_moduleStates, Constants.Swerve.MAX_MODULE_SPEED);

    adv_targetStatesPub.set(m_moduleStates);

    m_frontLeftWheel.set(m_moduleStates[0]);
    m_frontRightWheel.set(m_moduleStates[1]);
    m_backLeftWheel.set(m_moduleStates[2]);
    m_backRightWheel.set(m_moduleStates[3]);

    m_modulePositions = new SwerveModulePosition[] {
      m_frontLeftWheel.getModulePosition(),
      m_frontRightWheel.getModulePosition(),
      m_backLeftWheel.getModulePosition(),
      m_backRightWheel.getModulePosition(),
    };

    odomPose = m_poseEstimator.update(Rotation2d.fromDegrees(getGyroAngle()), m_modulePositions);
    SmartDashboard.putData("Gyro", m_gyro);

    adv_statesPub.set(new SwerveModuleState[]{
      m_frontLeftWheel.getModuleState(),
      m_frontRightWheel.getModuleState(),
      m_backLeftWheel.getModuleState(),
      m_backRightWheel.getModuleState()
    });

    m_field.setRobotPose(odomPose);

    adv_gyroPub.set(Rotation2d.fromDegrees(getGyroAngle()));
    adv_posePub.set(odomPose);
    SmartDashboard.putData("Field", m_field);

  }

  public double aimSwerveDrive(double xSpeed, double ySpeed, double targetX, double targetY){
    double target = Math.atan2(targetY - odomPose.getY(), targetX - odomPose.getX());
    return aimAngle(xSpeed, ySpeed, target);
  }

  public double aimReverse(double xSpeed, double ySpeed, double targetX, double targetY){
    double target = Math.atan2(odomPose.getY() - targetY, odomPose.getX() - targetX);
    return aimAngle(xSpeed, ySpeed, target);
  }

  public double aimAngle(double xSpeed, double ySpeed, double target){
    double angle = getGyroAngle() * Math.PI / 180.0;
    target = MathUtil.angleModulus(target);

    SmartDashboard.putNumber("AIMANGLE", angle);
    SmartDashboard.putNumber("AIMTARGET", target);
    double max = 2.0;
    double angSpeed = Math.max(Math.min(m_anglePID.calculate(angle, target), max), -max);
    SmartDashboard.putNumber("angPID", angSpeed);
    if (Math.abs(angSpeed) < 0.01){
      angSpeed = 0;
    }
    setSwerveDrive(xSpeed, ySpeed, angSpeed);
    return target;
  }

  public double[] inputSpeeds() {
    // double joystickX = m_mainStick.getRawAxis(1);
    // double joystickY = m_mainStick.getRawAxis(0);
    // double joystickA = m_mainStick.getRawAxis(4);
    double joystickX = (DriverStation.getAlliance().get() == Alliance.Red ? 1 : -1) * m_mainStick.getRawAxis(1);
    double joystickY = (DriverStation.getAlliance().get() == Alliance.Red ? 1 : -1) * m_mainStick.getRawAxis(0);
    // double joystickA = -m_mainStick.getRawAxis(2);
    double joystickA = -m_secondStick.getRawAxis(0);
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
    double spd = Constants.Swerve.MAX_SPEED * (m_mainStick.getRawButton(1) ? 0.5 : 1.0);
    x = Math.cos(dir) * Math.abs(Math.pow(mag, 3)) * spd;
    y = Math.sin(dir) * Math.abs(Math.pow(mag, 3)) * spd;
    a = Math.signum(a) * Math.abs(Math.pow(a, 3)) * Constants.Swerve.MAX_ANG_INPUT;

    SmartDashboard.putNumber("inX", x);
    SmartDashboard.putNumber("inY", y);
    SmartDashboard.putNumber("inA", a);

    // if (DriverStation.getAlliance().get() == Alliance.Red){
    //   x *= -1;
    //   y *= -1;
    // }
    
    return new double[]{x, y, a};

  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute", m_frontLeftCoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("regular", m_frontLeftCoder.getPosition().getValue());
    SmartDashboard.putBoolean("Gyro Rotating", m_gyro.isRotating());
    aimed = m_anglePID.getPositionError() < 0.1;
    SmartDashboard.putBoolean("Swerve Aimed", aimed);
  }
}