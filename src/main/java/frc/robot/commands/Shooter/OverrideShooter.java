package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class OverrideShooter extends Command {

    Shooter m_shooter;
    Joystick m_rightBoard;

    public OverrideShooter(Shooter m_shooter){
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
    }

    @Override
    public void initialize(){
        m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);
    }

    @Override
    public void execute(){
        double input = m_rightBoard.getRawAxis(Constants.IO.Board.Right.RIGHT_CLIMB);

        if (Math.abs(input) > 0.5){
            double a = Math.max(Math.min(m_shooter.setpoint - Math.signum(input) * 0.001, 0.16), 0.0);
            m_shooter.setpoint = a;
        }
    }

    @Override
    public void end(boolean isFinished){}
}
