package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class ResetGyro extends Command {
    DriveTrain m_drive;

    public ResetGyro(DriveTrain m_drive){
        addRequirements(m_drive);
        this.m_drive = m_drive;
    }

    @Override
    public void initialize(){

        m_drive.resetGyro();

    }

    @Override
    public boolean isFinished(){
        return true;
    }

    
}
