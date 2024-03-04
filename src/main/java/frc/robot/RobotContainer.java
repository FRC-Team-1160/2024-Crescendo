package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveTrain.SwerveDrive;
import frc.robot.commands.Shooter.IncrementShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Intake.Transport;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Commands

/*
import frc.robot.commands.drive.Drive;
import frc.robot.commands.vision.LimelightCameraToggle;
import frc.robot.commands.vision.LimelightLightToggle;
import frc.robot.commands.vision.LimelightSnapshotToggle;
import frc.robot.commands.vision.LimelightStreamToggle;
*/

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
    public final Vision m_vision = Vision.getInstance();
    public final Shooter m_shooter = Shooter.getInstance();
    public final Intake m_intake = Intake.getInstance();
    public final Climber m_climber = Climber.getInstance();
    public final Transport m_transport = Transport.getInstance();

    // Controllers
    private Joystick m_mainStick = new Joystick(0);
    private Joystick m_secondStick = new Joystick(1);



    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands

      m_driveTrain.setDefaultCommand(new SwerveDrive(m_driveTrain));

    }

      
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      // new JoystickButton(m_mainStick, Button.kA.value)
      //   .whileTrue(

      //   );

      // new JoystickButton(m_mainStick, Button.kStart.value)
      //   .onTrue(

      //   );

      /*ew JoystickButton(m_mainStick, Button.kX.value)
        .whenPressed(
          new ToggleGate(m_driveTrain)
        // );*/
        // new JoystickButton(m_mainStick, 2)
        //     .toggleOnTrue(new SwerveDriveShoot(m_driveTrain));

        // new JoystickButton(m_mainStick, 4)
        //     .toggleOnTrue(new IncrementShooter(m_shooter, 0.05)); 

        // new JoystickButton(m_mainStick, 1)
        //     .toggleOnTrue(new IncrementShooter(m_shooter, -0.05));
        
        new JoystickButton(m_mainStick, 14) //14
            .onTrue(new InstantCommand(() -> m_driveTrain.resetGyro(), m_driveTrain));

        new JoystickButton(m_secondStick, 3)
            .onTrue(new InstantCommand(() -> m_shooter.setSpeed(0), m_shooter));

        new JoystickButton(m_secondStick, 2)
            .onTrue(new InstantCommand(() -> m_shooter.setSpeed(0.5), m_shooter));

        new JoystickButton(m_secondStick, 4)
            .onTrue(new IncrementShooter(m_shooter, 0.05));

        new JoystickButton(m_secondStick, 1)
            .onTrue(new IncrementShooter(m_shooter, -0.05));

        new JoystickButton(m_secondStick, 5)
            .onTrue(new InstantCommand(() -> m_intake.setSolenoid(1), m_intake));
        
        new JoystickButton(m_secondStick, 6)
            .onTrue(new InstantCommand(() -> m_intake.toggleSolenoid(), m_intake));

        new JoystickButton(m_secondStick, 7)
            .onTrue(new InstantCommand(() -> m_intake.toggleMotor(), m_intake)
                .andThen(new InstantCommand(() -> m_transport.toggleWheels(), m_transport)));

        new JoystickButton(m_secondStick, 8)
            .onTrue(new InstantCommand(() -> m_transport.toggleBelt(), m_transport));

        new JoystickButton(m_secondStick, 10)
            .onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> m_intake.toggleMotor(), m_intake),
                new InstantCommand(() -> m_transport.toggleWheels(), m_transport),
                new InstantCommand(() -> m_transport.toggleBelt(), m_transport)
                )
            );
        
        // new JoystickButton(m_mainStick, 8)
        //     .toggleOnTrue(new SwerveDriveSpeaker(m_driveTrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    
    public Command getAutonomousCommand() {
        return null;
    }
    
}