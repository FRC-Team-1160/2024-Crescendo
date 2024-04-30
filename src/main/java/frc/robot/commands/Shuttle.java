// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Intake.Transport;

public class Shuttle extends Command {
  /** Creates a new SwerveDrive. */
  private double x;
  private double y;
  double a;
  DriveTrain m_drive;
  Shooter m_shooter;
  double target_x;
  double target_z;
  boolean aim;

  public Shuttle(DriveTrain m_drive, Shooter m_shooter, boolean aim) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_shooter);
    this.m_drive = m_drive;
    this.m_shooter = m_shooter;
    this.aim = aim;
  }
  
  /**
   * @param m_drive
   * @param m_shooter
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.blinkin.set(-0.09);
    target_x = 0.5;
    target_z = 2.05;
  }

  @Override
  public void execute(){

    double[] inputs = m_drive.inputSpeeds();
    x = inputs[0];
    y = inputs[1];
    a = inputs[2];

    SmartDashboard.putNumber("Forward", x);
    SmartDashboard.putNumber("Sideways", y);

    // double step = 2.0;

    double back_x = 2.0;
    if (m_drive.isRed) {
      target_x = Constants.Field.FIELD_LENGTH - target_x;
      back_x = Constants.Field.FIELD_LENGTH - back_x;
    }
    double pitch = m_shooter.degToSetpoint(60);
    SmartDashboard.putNumber("sixty deg in enc", pitch);
    
    if (aim) {
        m_drive.aimSwerveDrive(x, y, (back_x), (Constants.Field.SPEAKER_Y + 2));
        double d = Math.sqrt(Math.pow(m_drive.odomPose.getX() - (back_x), 2) + Math.pow(m_drive.odomPose.getY() - (Constants.Field.SPEAKER_Y + 2), 2));
        SmartDashboard.putNumber("Dist", d);
        pitch = 0.16 - 0.004 * d;
    } else {
        m_drive.setSwerveDrive(x, y, a);
    }
    SmartDashboard.putNumber("Shooter Aim", pitch);
    m_shooter.setpoint = MathUtil.clamp(pitch, 0.11, 0.16);
    SmartDashboard.putNumber("Shooter Rev", m_shooter.setSpeed(0.65));

    if (m_shooter.revved && m_shooter.aimed && m_drive.aimed){
      m_shooter.blinkin.set(0.93);
    } else {
      m_shooter.blinkin.set(-0.09);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Shooter Speed", 0);
    m_shooter.setSpeed(0);
    if (Transport.getInstance().noteStored){
      m_shooter.blinkin.set(0.85);
    } else {
      m_shooter.blinkin.set(0.93);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
    