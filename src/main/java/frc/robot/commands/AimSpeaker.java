// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class AimSpeaker extends Command {
  /** Creates a new SwerveDrive. */
  private double x;
  private double y;
  DriveTrain m_drive;
  Shooter m_shooter;

  public AimSpeaker(DriveTrain m_drive, Shooter m_shooter) {
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

    double[] inputs = m_drive.inputSpeeds();
    x = inputs[0];
    y = inputs[1];

    SmartDashboard.putNumber("Forward", x);
    SmartDashboard.putNumber("Sideways", y);

    m_drive.aimSwerveDrive(x, y, 16.54 + 0.1, 5.5);

    // if (m_mainStick.getRawButton(1) || RobotBase.isSimulation()){
    //   m_drive.aimSwerveDrive(x, y, 0.5, 5.5);
    // } else {
    //   m_drive.setSwerveDrive(0, 0, 0);
    // }
    double x = SmartDashboard.getNumber("GetX", 16.54);
    double z = SmartDashboard.getNumber("GetZ", 2.0);
    SmartDashboard.putNumber("Shooter Aim", m_shooter.aimTarget(x, 5.5, z));
    SmartDashboard.putNumber("Shooter Rev", m_shooter.revTarget(16.54, 5.5));

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Shooter Speed", 0);
    m_shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
    