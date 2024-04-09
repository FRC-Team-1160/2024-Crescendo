// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class AimSpeakerAuto extends Command {
  /** Creates a new SwerveDrive. */
  private double x;
  private double y;
  DriveTrain m_drive;
  Shooter m_shooter;
  public double target_a;

  public AimSpeakerAuto(DriveTrain m_drive, Shooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_shooter);
    this.m_drive = m_drive;
    this.m_shooter = m_shooter;
    
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
    double target_x = 0.5;
    double target_z = 2.15;
    double back_x = 0.0;

    if (m_drive.odomPose.getX() > 8.25) {
      target_x = 16.5 - target_x;
      back_x = 16.5 - back_x;
    }
    
    target_a = m_drive.aimSwerveDrive(x, y, back_x, 5.5);

    // SmartDashboard.putNumber("Shooter Aim", m_shooter.aimTarget(x, 5.5, z));
    // SmartDashboard.putNumber("Shooter Rev", m_shooter.revTarget(16.54, 5.5));
    SmartDashboard.putNumber("Shooter Aim", m_shooter.aimTarget(target_x, 5.5, target_z + m_shooter.offset));
    SmartDashboard.putNumber("Shooter Rev", m_shooter.revTarget(back_x, 5.5));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putNumber("Shooter Speed", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double ang_diff = m_drive.m_anglePID.getPositionError();
    
    return Math.abs(ang_diff) < 5;
  }
}
    