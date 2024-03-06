package frc.robot;

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

  public static final class RobotConstants {
    public static final double MAX_VEL = 0;
    public static final double MAX_ACEL = 0;
    public static final double MAX_ANG_VEL = 0;
    public static final double MAX_ANG_ACEL = 0;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        MAX_ANG_ACEL,
                        MAX_ANG_ACEL);
  }

  public static final class PortConstants {
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

    public static final int SHOOTER_TOP_MOTOR = 9;
    public static final int SHOOTER_BOTTOM_MOTOR = 2;
    public static final int SHOOTER_PITCH_MOTOR = 7;

    public static final int INTAKE_MOTOR = 4;

    public static final int TRANSPORT_LEFT_MOTOR = 10;
    public static final int TRANSPORT_RIGHT_MOTOR = 6;
    public static final int TRANSPORT_BELT_MOTOR = 5;
    
    public static final int TRANSPORT_ULTRASONIC = 0;
    
    public static final int CLIMBER_LEFT_MOTOR = 8;
    public static final int CLIMBER_RIGHT_MOTOR = 3;

  }

  public static final class InputConstants {
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
}