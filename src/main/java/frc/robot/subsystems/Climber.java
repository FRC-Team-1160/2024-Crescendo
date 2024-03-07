// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Climber extends SubsystemBase {
  
  private static Climber m_instance;

  public PIDController leftPID;
  public PIDController rightPID;

  public CANSparkMax left;
  public CANSparkMax right;

  public double left_setpoint;
  public double right_setpoint;

  private Joystick m_secondStick;

  
  public static Climber getInstance(){
    if (m_instance == null){
      m_instance = new Climber();
    }
    return m_instance;
  }
  
  public Climber() {
    left = new CANSparkMax(PortConstants.CLIMBER_LEFT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    right = new CANSparkMax(PortConstants.CLIMBER_RIGHT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    leftPID = new PIDController(0.01, 0, 0);
    rightPID = new PIDController(0.01, 0, 0);
    left_setpoint = left.getEncoder().getPosition();
    right_setpoint = right.getEncoder().getPosition();

    m_secondStick = new Joystick(1);
  }


  @Override
  public void periodic() {
    if (Math.abs(m_secondStick.getRawAxis(1)) > 0.8){
      left.set(-Math.signum(m_secondStick.getRawAxis(1)) * 0.4);
    } else {
      left.set(0);
    }
    if (Math.abs(m_secondStick.getRawAxis(5)) > 0.8){
      right.set(Math.signum(m_secondStick.getRawAxis(5)) * 0.4);
    } else {
      right.set(0);
    }
  }
}
