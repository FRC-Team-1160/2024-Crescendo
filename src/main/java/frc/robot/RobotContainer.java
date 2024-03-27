package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimSpeaker;
import frc.robot.commands.AimSpeakerAuto;
import frc.robot.commands.DriveTrain.SwerveDrive;
import frc.robot.commands.DriveTrain.SwerveDriveMoveAuto;
import frc.robot.commands.Intake.IntakeAuto;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Intake.OuttakeNote;
import frc.robot.commands.Shooter.AmpPreset;
import frc.robot.commands.Shooter.OverrideShooter;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Transport;
import frc.robot.subsystems.Vision.Vision;

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

    public SendableChooser<String> m_pathChooser;
    public SendableChooser<Command> m_posChooser;

    private Joystick m_mainStick = new Joystick(Constants.IO.MAIN_PORT);
    private Joystick m_codriverStick = new Joystick(Constants.IO.COPILOT_PORT);
    private Joystick m_codriverSimpStick = new Joystick(Constants.IO.COPILOT_SIMP_PORT);
    private Joystick m_leftBoard = new Joystick(Constants.IO.LEFT_BOARD_PORT);
    private Joystick m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);

    public boolean isRedAlliance = (DriverStation.getAlliance().get() == Alliance.Red);
    final double forward = isRedAlliance ? 180 : 0;
    final double backward = isRedAlliance ? 0 : 180;


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public Pose2d flipPose(Pose2d pose){
      return (isRedAlliance ? new Pose2d(16.54 - pose.getX(), pose.getY(), Rotation2d.fromDegrees(180 - pose.getRotation().getDegrees())) : pose);
    }

    public RobotContainer() {
      configureButtonBindings();
      

      m_pathChooser = new SendableChooser<>();
      m_pathChooser.setDefaultOption("AFK", "AFK");
      m_pathChooser.addOption("Park", "Park");
      m_pathChooser.addOption("Two Note", "Two Note");
      m_pathChooser.addOption("Disruptor", "Disruptor");
      m_pathChooser.addOption("Test", "Test");

      m_posChooser = buildAutoChooser(m_pathChooser.getSelected());
      SmartDashboard.putData("Auto Chooser", m_pathChooser);
      SmartDashboard.putData("Auto Start Position", m_posChooser);

      m_driveTrain.setDefaultCommand(new SwerveDrive(m_driveTrain));
      m_intake.setDefaultCommand(new InstantCommand(() -> m_intake.m_solenoid.set(m_intake.solenoid_default), m_intake));
      // m_shooter.setDefaultCommand(new InstantCommand(() -> m_shooter.setSpeed(0.0), m_shooter));

    }
    
    private void configureButtonBindings() {

        new JoystickButton(m_mainStick, 14)
            .onTrue(new InstantCommand(() -> m_driveTrain.resetGyro()));

        new JoystickButton(m_codriverStick, 1)
            .whileTrue(new RunCommand(() -> m_driveTrain.aimAngle(m_driveTrain.inputSpeeds()[0], m_driveTrain.inputSpeeds()[1], backward * Math.PI/180), m_driveTrain));
        
        new JoystickButton(m_leftBoard, Constants.IO.Board.Left.SHOOT)
        .or(new JoystickButton(m_leftBoard, Constants.IO.Board.Left.SHOOT_OVERRIDE))
            .onTrue(new Shoot(m_shooter, m_transport));
        
        new JoystickButton(m_leftBoard, Constants.IO.Board.Left.AIM)
            .whileTrue(new AimSpeaker(m_driveTrain, m_shooter));

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
        
        new JoystickButton(m_rightBoard, Constants.IO.Board.Right.OVERRIDE)
            .whileTrue(new OverrideShooter(m_shooter));
        
        new JoystickButton(m_rightBoard, Constants.IO.Board.Right.MOVE_TAR)
            .onTrue(new InstantCommand(() -> m_shooter.offset += m_rightBoard.getRawButton(Constants.IO.Board.Right.INC_OR_DEC_TAR) ? 0.04 : -0.04));

        new JoystickButton(m_codriverSimpStick, 1)
            .whileTrue(new AimSpeaker(m_driveTrain, m_shooter));

        new JoystickButton(m_codriverSimpStick, 2)
            .onTrue(new Shoot(m_shooter, m_transport));

        new JoystickButton(m_codriverSimpStick, 4)
            .onTrue(new IntakeNote(m_intake, m_transport));

        new JoystickButton(m_codriverSimpStick, 3)
            .onTrue(new InstantCommand(() -> m_shooter.setSpeed(0.25)));
        new JoystickButton(m_codriverSimpStick, 3)
            .onFalse(new InstantCommand(() -> m_shooter.setSpeed(0.0)));
        new JoystickButton(m_codriverSimpStick, 5)
            .onTrue(new InstantCommand(() -> m_shooter.setSpeed(0.5)));
        new JoystickButton(m_codriverSimpStick, 5)
            .onFalse(new InstantCommand(() -> m_shooter.setSpeed(0.0)));
        // new JoystickButton(m_mainStick, 8)
        //     .toggleOnTrue(new SwerveDriveSpeaker(m_driveTrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    
    public Command getAutonomousCommand() {
      return m_posChooser.getSelected();
    }

    public SendableChooser<Command> buildAutoChooser(String path) {
      SendableChooser<Command> chooser = new SendableChooser<>();
      switch(path){
        case "AFK":
          chooser.addOption("Sub1", new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.SUB1))));
          chooser.addOption("Sub2", new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.SUB2))));
          chooser.addOption("Sub3", new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.SUB3))));
          chooser.addOption("Wall", new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.WALL))));
          break;
        case "Park":
          chooser.addOption("Sub1", new SequentialCommandGroup(
            new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.SUB1))),
            new SwerveDriveMoveAuto(m_driveTrain, 2.7, Constants.Auto.Start.SUB1.getY(), null)

          ));
          chooser.addOption("Sub2", new SequentialCommandGroup(
            new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.SUB2))),
            new SwerveDriveMoveAuto(m_driveTrain, 2.7, Constants.Auto.Start.SUB2.getY(), null)
          ));
          chooser.addOption("Sub3", new SequentialCommandGroup(
            new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.SUB3))),
            new SwerveDriveMoveAuto(m_driveTrain, 2.5, Constants.Auto.Start.SUB3.getY(), null)
          ));
          chooser.addOption("Wall", new SequentialCommandGroup(
            new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.WALL))),
            new SwerveDriveMoveAuto(m_driveTrain, 2.7, Constants.Auto.Start.WALL.getY(), null)
          ));
          break;
        case "Two Note":
          chooser.addOption("Sub1", buildTwoNote(
            Constants.Auto.Start.SUB1, 
            new Translation2d(1.8, 7.0),
            new Translation2d(2.7, 7.0)
            ));
          chooser.addOption("Sub2", buildTwoNote(
            Constants.Auto.Start.SUB2, 
            new Translation2d(2.0, 5.5),
            new Translation2d(2.7, 5.5)
            ));
          chooser.addOption("Sub3", buildTwoNote(
            Constants.Auto.Start.SUB3, 
            new Translation2d(1.8, 4.0),
            new Translation2d(2.5, 4.0)
            ));
          break;
        case "Disruptor":
          chooser.addOption("Wall", new SequentialCommandGroup(
            new InstantCommand(() -> m_driveTrain.resetPose(flipPose(Constants.Auto.Start.WALL))),
            new SwerveDriveMoveAuto(m_driveTrain, 8.6, 0.8, 135.0,
            new TrapezoidProfile.Constraints(3.0, 5.0), 0.004),
            new SwerveDriveMoveAuto(m_driveTrain, 8.6, 7.5,null,
            new TrapezoidProfile.Constraints(3.0, 5.0), 0.004)
          ));
          break;
        case "Test":
          chooser.addOption("Square", new PathPlannerAuto("Square"));
          chooser.addOption("Spin", new PathPlannerAuto("Spin"));
      }
      return chooser;
    }
    public Command buildTwoNote(Pose2d start, Translation2d pos1, Translation2d pos2) {
      return new ParallelCommandGroup(new WaitCommand(15.0),
        new SequentialCommandGroup(
          new InstantCommand(() -> m_driveTrain.resetPose(flipPose(start))),
          new SwerveDriveMoveAuto(m_driveTrain, pos1.getX(), pos1.getY(), null),
          new AimSpeakerAuto(m_driveTrain, m_shooter),
          new WaitCommand(0.5),
          new Shoot(m_shooter, m_transport),
          new ParallelCommandGroup(
            new IntakeAuto(m_intake, m_transport),
            new SwerveDriveMoveAuto(m_driveTrain, pos2.getX(), pos2.getY(), 180.0)
          ),
          new AimSpeakerAuto(m_driveTrain, m_shooter),
          new WaitCommand(0.5),
          new Shoot(m_shooter, m_transport)
          )
        );
      
    }
    
}