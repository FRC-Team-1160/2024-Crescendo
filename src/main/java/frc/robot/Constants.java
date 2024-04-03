package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Units are m kg s unless otherwise specified
  public static final class Swerve {
    public static final double WHEEL_DIAMETER = 4 * 0.0254 * Math.PI;
    public static final double GEAR_RATIO = 6.75;
    public static final double MAX_SPEED = 3.0;
    public static final double MAX_MODULE_SPEED = 5.0;
    public static final double SIDE_LENGTH = 23.5 * 0.0254;
    public static final double BASE_RADIUS = SIDE_LENGTH / Math.sqrt(2);
    public static final double MAX_ANG_SPEED = MAX_MODULE_SPEED / BASE_RADIUS;
  }

  public static final class Field {
    public static final double FIELD_LENGTH = 16.54;
    public static final double SPEAKER_Y = 5.6;
  }

  public static final class Auto {
    public static final double MAX_VEL = 1.0;
    public static final double MAX_ACEL = 1.5;
    public static final double MAX_ANG_VEL = 90;
    public static final double MAX_ANG_ACEL = 90;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
        MAX_ANG_VEL,
        MAX_ANG_ACEL
      );
    public static final TrapezoidProfile.Constraints kVelocityControllerConstraints =
      new TrapezoidProfile.Constraints(
        MAX_VEL,
        MAX_ACEL
      );

    public static final class Start {
      public static final Pose2d SUB1 = new Pose2d(0.8, 6.7, Rotation2d.fromDegrees(120));
      public static final Pose2d SUB2 = new Pose2d(1.3, 5.5, Rotation2d.fromDegrees(180));
      public static final Pose2d SUB3 = new Pose2d(0.8, 4.5, Rotation2d.fromDegrees(-120));
      public static final Pose2d WALL = new Pose2d(0.5, 1.5, Rotation2d.fromDegrees(180));
    }
  }

  public static final class Port {
    // CAN ID 
    public static final int FRONT_LEFT_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_STEER_MOTOR = 3;
    public static final int BACK_LEFT_STEER_MOTOR = 5;
    public static final int BACK_RIGHT_STEER_MOTOR = 7;

    public static final int FRONT_LEFT_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_DRIVE_MOTOR = 8;

    public static final int FRONT_LEFT_CODER = 1;
    public static final int FRONT_RIGHT_CODER = 3;
    public static final int BACK_LEFT_CODER = 5;
    public static final int BACK_RIGHT_CODER = 7;

    public static final int SHOOTER_TOP_MOTOR = 10;
    public static final int SHOOTER_BOTTOM_MOTOR = 9;
    public static final int SHOOTER_PITCH_MOTOR = 7;

    public static final int INTAKE_MOTOR = 4;

    public static final int TRANSPORT_LEFT_MOTOR = 10;
    public static final int TRANSPORT_RIGHT_MOTOR = 6;
    public static final int TRANSPORT_BELT_MOTOR = 5;
    
    public static final int TRANSPORT_ULTRASONIC = 0;
    
    public static final int CLIMBER_LEFT_MOTOR = 8;
    public static final int CLIMBER_RIGHT_MOTOR = 3;
    
    public static final int LEFT_CLIMB_LIMIT = 0;
    public static final int RIGHT_CLIMB_LIMIT = 1;
    public static final int TRANSPORT_LIMIT = 2;

  }

  public static final class IO {
    public static final int MAIN_PORT = 0;
    public static final int COPILOT_PORT = 1;
    public static final int COPILOT_SIMP_PORT = 2;
    public static final int LEFT_BOARD_PORT = 3;
    public static final int RIGHT_BOARD_PORT = 4;
    public static final class Logitech {
      public static final int LX_AXIS = 0;
      public static final int LY_AXIS = 1;
      public static final int RT_AXIS = 2;
      public static final int LT_AXIS = 3;
      public static final int RX_AXIS = 4;
      public static final int RY_AXIS = 5;

      public static final int A = 1;
      public static final int B = 2;
      public static final int X = 3;
      public static final int Y = 4;
      public static final int LB = 5;
      public static final int RB = 6;
      public static final int BACK = 7;
      public static final int START = 8;
      public static final int LS = 9;
      public static final int RS = 10;
    }

    public static final class Board {
      public static final class Left {
        public static final int SHOOT = 1;
        public static final int AIM = 2;

        public static final int AMP = 3;

        public static final int SHOOT_OVERRIDE = 5;
        public static final int REV = 6;

        public static final int LEFT_CLIMB = 0;
      }
      public static final class Right {
        public static final int UP_DOWN_INTAKE = 1;
        public static final int OVERRIDE = 4;
        public static final int OUTTAKE = 9;
        public static final int INTAKE = 8;

        public static final int INC_OR_DEC_TAR = 3;
        public static final int MOVE_TAR = 6;

        public static final int RIGHT_CLIMB = 0;
      }
    }
  }
}