package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimSpeaker;
import frc.robot.commands.Intake.OuttakeNote;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.DriveTrain.SwerveDrive;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.AmpPreset;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Transport;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
    public final Vision m_vision = Vision.getInstance();
    public final Shooter m_shooter = Shooter.getInstance();
    public final Intake m_intake = Intake.getInstance();
    public final Climber m_climber = Climber.getInstance();
    public final Transport m_transport = Transport.getInstance();

    private final SendableChooser<Command> autoChooser;

    private Joystick m_mainStick = new Joystick(Constants.IO.MAIN_PORT);
    private Joystick m_codriverStick = new Joystick(Constants.IO.COPILOT_PORT);
    private Joystick m_codriverSimpStick = new Joystick(Constants.IO.COPILOT_SIMP_PORT);
    private Joystick m_leftBoard = new Joystick(Constants.IO.LEFT_BOARD_PORT);
    private Joystick m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      configureButtonBindings();
      autoChooser = AutoBuilder.buildAutoChooser();
      m_driveTrain.setDefaultCommand(new SwerveDrive(m_driveTrain));
      m_intake.setDefaultCommand(new InstantCommand(() -> m_intake.m_solenoid.set(m_intake.solenoid_default), m_intake));

    }
    
    private void configureButtonBindings() {

        new JoystickButton(m_mainStick, 14)
            .onTrue(new InstantCommand(() -> m_driveTrain.resetGyro()));
        new JoystickButton(m_leftBoard, Constants.IO.Board.Left.SHOOT)
        .or(new JoystickButton(m_leftBoard, Constants.IO.Board.Left.SHOOT_OVERRIDE))
            .onTrue(new Shoot(m_shooter, m_transport));
        
        new JoystickButton(m_leftBoard, Constants.IO.Board.Left.AIM)
            .onTrue(new AimSpeaker(m_driveTrain, m_shooter));

        new JoystickButton(m_leftBoard, Constants.IO.Board.Left.AMP)
            .toggleOnTrue(new AmpPreset(m_shooter));

        new JoystickButton(m_leftBoard, Constants.IO.Board.Left.REV)
            .onTrue(new InstantCommand(() -> m_shooter.setSpeed(0.8)));
        new JoystickButton(m_leftBoard, Constants.IO.Board.Left.REV)
            .onFalse(new InstantCommand(() -> m_shooter.setSpeed(0)));

        new JoystickButton(m_rightBoard, Constants.IO.Board.Right.INTAKE)
            .toggleOnTrue(new IntakeNote(m_intake, m_transport));
        new JoystickButton(m_rightBoard, Constants.IO.Board.Right.OUTTAKE)
            .toggleOnTrue(new OuttakeNote(m_intake, m_transport, m_shooter));

        new JoystickButton(m_rightBoard, Constants.IO.Board.Right.UP_DOWN_INTAKE)
            .onTrue(new InstantCommand(() -> m_intake.solenoid_default = DoubleSolenoid.Value.kForward));
        new JoystickButton(m_rightBoard, Constants.IO.Board.Right.UP_DOWN_INTAKE)
            .onFalse(new InstantCommand(() -> m_intake.solenoid_default = DoubleSolenoid.Value.kReverse));

        new JoystickButton(m_codriverSimpStick, 1)
        .whileTrue(new AimSpeaker(m_driveTrain, m_shooter));

        new JoystickButton(m_codriverSimpStick, 2)
            .onTrue(new Shoot(m_shooter, m_transport));

        new JoystickButton(m_codriverSimpStick, 4)
            .onTrue(new IntakeNote(m_intake, m_transport));

        new JoystickButton(m_codriverSimpStick, 3)
            .onTrue(new InstantCommand(() -> m_shooter.setSpeed(0.25)));
        
        // new JoystickButton(m_mainStick, 8)
        //     .toggleOnTrue(new SwerveDriveSpeaker(m_driveTrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}