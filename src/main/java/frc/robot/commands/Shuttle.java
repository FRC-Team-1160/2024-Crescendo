// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  DriveTrain m_drive;
  Shooter m_shooter;
  double target_x;
  double target_z;
  int whereToAim;

  public Shuttle(DriveTrain m_drive, Shooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_shooter);
    this.m_drive = m_drive;
    this.m_shooter = m_shooter;
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

    SmartDashboard.putNumber("Forward", x);
    SmartDashboard.putNumber("Sideways", y);

    // m_drive.aimSwerveDrive(x, y, 16.54 + 0.1, 5.5);
    double step = 2.0;

    double back_x = 0;
    if ((m_drive.isRed) || (this.whereToAim == -1)) {
      target_x = Constants.Field.FIELD_LENGTH - target_x;
      back_x = Constants.Field.FIELD_LENGTH - back_x;
    }
    
    m_drive.aimSwerveDrive(x, y, (back_x - x*step), (Constants.Field.SPEAKER_Y - y*step));

    if (Math.abs(this.whereToAim) == 0) {
      // SmartDashboard.putNumber("Shooter Aim", m_shooter.aimTarget(x, 5.5, target_z));
      // SmartDashboard.putNumber("Shooter Rev", m_shooter.revTarget(16.54, 5.5));
      SmartDashboard.putNumber("Shooter Aim", m_shooter.aimTarget(target_x - x*step, Constants.Field.SPEAKER_Y - y*step, target_z + m_shooter.offset));
      SmartDashboard.putNumber("Shooter Rev", m_shooter.revTarget(back_x - x*step, Constants.Field.SPEAKER_Y - y*step));
    }

    if (m_shooter.revved && m_shooter.aimed){
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
    