package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveTrain.SwerveDrive;
import frc.robot.commands.DriveTrain.SwerveDriveSpeakerAuto;
import frc.robot.commands.Intake.IntakeRun;
import frc.robot.commands.Shooter.IncrementShooter;
import frc.robot.commands.Shooter.SetShooter;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Intake.Transport;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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

    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
    public final Vision m_vision = Vision.getInstance();
    public final Shooter m_shooter = Shooter.getInstance();
    public final Intake m_intake = Intake.getInstance();
    public final Climber m_climber = Climber.getInstance();
    public final Transport m_transport = Transport.getInstance();

    private final SendableChooser<Command> autoChooser;

    private Joystick m_mainStick = new Joystick(0);
    private Joystick m_secondStick = new Joystick(1);

    public RobotContainer() {
      configureButtonBindings();
      autoChooser = AutoBuilder.buildAutoChooser();
      m_driveTrain.setDefaultCommand(new SwerveDrive(m_driveTrain));

    }

    
    public Command IntakeRun() {
        return new IntakeRun(m_intake);
    }
    public Command SwerveDriveSpeakerAuto() {
        return new SwerveDriveSpeakerAuto(m_driveTrain);
    }
    public Command Rev() {
        return new SetShooter(m_shooter, 0.6);
    }
    public Command Shoot() {
        return new Shoot(m_shooter, 0.6);
    } 
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
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}