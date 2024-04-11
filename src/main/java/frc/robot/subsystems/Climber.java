// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.Port;

public class Climber extends SubsystemBase {
  
  private static Climber m_instance;

  public PIDController leftPID;
  public PIDController rightPID;

  public CANSparkMax left;
  public CANSparkMax right;

  public double left_setpoint;
  public double right_setpoint;

  // private Joystick m_secondStick;
  private Joystick m_leftBoard;
  private Joystick m_rightBoard;

  public DigitalInput l_limit;
  public DigitalInput r_limit;
  
  public static Climber getInstance(){
    if (m_instance == null){
      m_instance = new Climber();
    }
    return m_instance;
  }
  
  public Climber() {
    left = new CANSparkMax(Port.CLIMBER_LEFT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    right = new CANSparkMax(Port.CLIMBER_RIGHT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    leftPID = new PIDController(0.01, 0, 0);
    rightPID = new PIDController(0.01, 0, 0);
    left_setpoint = left.getEncoder().getPosition();
    right_setpoint = right.getEncoder().getPosition();

    // m_secondStick = new Joystick(1);
    m_leftBoard = new Joystick(Constants.IO.LEFT_BOARD_PORT);
    m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);

    l_limit = new DigitalInput(Constants.Port.LEFT_CLIMB_LIMIT);
    r_limit = new DigitalInput(Constants.Port.RIGHT_CLIMB_LIMIT);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("LClimber", left.getEncoder().getPosition());
    SmartDashboard.putNumber("RClimber", right.getEncoder().getPosition());
    // double l_input = m_simpStick.getRawAxis(1);
    // double r_input = m_simpStick.getRawAxis(3);
    double l_input = Math.min(m_leftBoard.getRawAxis(Constants.IO.Board.Left.LEFT_CLIMB), l_limit.get() ? 0 : 1);
    double r_input = Math.min(m_rightBoard.getRawAxis(Constants.IO.Board.Right.RIGHT_CLIMB), r_limit.get() ? 0 : 1);
    boolean override = new JoystickButton(m_rightBoard, Constants.IO.Board.Right.OVERRIDE).getAsBoolean();

    if (Math.abs(l_input) > 0.8){
      left.set(-Math.signum(l_input) * 1);
    } else {
      left.set(0);
    }
    if (Math.abs(r_input) > 0.8 && !override){
      right.set(Math.signum(r_input) * 1);
    } else {
      right.set(0);
    }
    SmartDashboard.putBoolean("limitL", l_limit.get());
    SmartDashboard.putBoolean("limitR", r_limit.get());

  }
}