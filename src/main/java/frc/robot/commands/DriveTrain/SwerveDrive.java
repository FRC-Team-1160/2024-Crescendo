// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveTrain;

// import edu.wpi.first.wpilibj.RobotBase;

public class SwerveDrive extends Command {
  /** Creates a new SwerveDrive. */
  private double x;
  private double y;
  private double a;
  DriveTrain m_drive;
  private Joystick m_mainStick = new Joystick(0);

  public SwerveDrive(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] inputs = m_drive.inputSpeeds();
    x = inputs[0];
    y = inputs[1];
    a = inputs[2];

    if (m_mainStick.getRawButton(1) || RobotBase.isSimulation()){
      m_drive.setSwerveDrive(x, y, a);
      //System.out.println(x);
    } else {
      m_drive.setSwerveDrive(0, 0, 0);
    }


    //System.out.println("def");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
    