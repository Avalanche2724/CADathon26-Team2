
package frc.robot;

public final class Constants {

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class SwerveConstants {

    // Front Left
    public static final int FL_DRIVE_ID = 1;
    public static final int FL_STEER_ID = 2;
    public static final int FL_ENCODER_ID = 3;

    // Front Right
    public static final int FR_DRIVE_ID = 4;
    public static final int FR_STEER_ID = 5;
    public static final int FR_ENCODER_ID = 6;

    // Back Left
    public static final int BL_DRIVE_ID = 7;
    public static final int BL_STEER_ID = 8;
    public static final int BL_ENCODER_ID = 9;

    // Back Right
    public static final int BR_DRIVE_ID = 10;
    public static final int BR_STEER_ID = 11;
    public static final int BR_ENCODER_ID = 12;

    // Hardware measurements
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double DRIVE_GEAR_RATIO = 6.03;
    public static final double STEER_GEAR_RATIO = 26.09;

    // Speed limits
    public static final double MAX_SPEED = 4.0;
    public static final double MAX_ANGULAR_SPEED = 8.0;
  }

  // Two intake roller motors
  public static class IntakeConstants {
    public static final int LEFT_MOTOR_ID = 13;
    public static final int RIGHT_MOTOR_ID = 14;

    public static final double INTAKE_SPEED = 0.5;
  }

  // Intake pivot motor
  public static class PivotConstants {
    public static final int MOTOR_ID = 15;
    public static final double PIVOT_SPEED = 0.2;
  }
}
