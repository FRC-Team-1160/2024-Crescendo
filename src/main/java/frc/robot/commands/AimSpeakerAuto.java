// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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

    double[] inputs = m_drive.inputSpeeds();
    x = inputs[0];
    y = inputs[1];

    double target_x = 0.3;
    double target_z = 1.95;
    double back_x = 0.0;

    if (m_drive.odomPose.getX() > 8.25){
      target_x = 16.54 - target_x;
      back_x = 16.54 - back_x;
    }
    target_a = m_drive.aimSwerveDrive(x, y, back_x, Constants.Field.SPEAKER_Y);

    m_shooter.aimTarget(target_x, Constants.Field.SPEAKER_Y, target_z);
    m_shooter.revTarget(back_x, Constants.Field.SPEAKER_Y);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putNumber("Shooter Speed", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return ((m_shooter.revved && m_shooter.aimed) || RobotBase.isSimulation());
  }
}
    