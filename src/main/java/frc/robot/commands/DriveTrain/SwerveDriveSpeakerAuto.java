// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class SwerveDriveSpeakerAuto extends Command {
  /** Creates a new SwerveDrive. */
  private double x;
  private double y;
  DriveTrain m_drive;
  public double target;

  public SwerveDriveSpeakerAuto(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // double joystickX = m_mainStick.getRawAxis(5);
    // double joystickY = m_mainStick.getRawAxis(4);
    // double joystickA = -m_mainStick.getRawAxis(0);
  }

  @Override
  public void execute(){

    double[] inputs = m_drive.inputSpeeds();
    x = inputs[0];
    y = inputs[1];

    SmartDashboard.putNumber("Forward", x);
    SmartDashboard.putNumber("Sideways", y);

    target = m_drive.aimSwerveDrive(x, y, 0.5, 5.5);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return target - DriveTrain.getInstance().getGyroAngle()/180.0*Math.PI <= 0.01;
  }
}
    