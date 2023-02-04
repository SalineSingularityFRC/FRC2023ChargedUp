package frc.robot;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {
    public static final double DRIVE_MOTOR_GEAR_RATIO = 8.14; // this is the ratio of bot
    public static final double ANGLE_MOTOR_GEAR_RATIO = 12.8; // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    public static final double MAX_ANGLE_INACCURACY = Math.PI/ 24;
    public static final String CANBUS = "rio";
    public static final String CANIVORE = "drivetrain";

    // --------->  THESE ARE REAL ID VALUES <------------
    public static final int GYRO_CANCODER_ID = 0;

    public static final int FL_ANGLE_ID = 11;
    public static final int FL_Motor_ID = 13;
    public static final int FL_CANCODER_ID = 42;
    public static final boolean FL_isInverted = false;

    public static final int FR_ANGLE_ID = 18;
    public static final int FR_Motor_ID = 14;
    public static final int FR_CANCODER_ID = 41;
    public static final boolean FR_isInverted = true;

    public static final int BL_ANGLE_ID = 16;
    public static final int BL_Motor_ID = 12;
    public static final int BL_CANCODER_ID = 44;
    public static final boolean BL_isInverted = false;

    public static final int BR_ANGLE_ID = 15;
    public static final int BR_Motor_ID = 17;
    public static final int BR_CANCODER_ID = 43;
    public static final boolean BR_isInverted = true;

    public static final int BA_Motor_ID = 6;

    public static double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = (0.339111) * 2 * Math.PI;
    public static double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = (0.148682) * 2 * Math.PI;
    public static double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = (0.800537) * 2 * Math.PI;
    public static double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = (0.823975) * 2 * Math.PI;

    // trackWidth - lateral distance between pairs of wheels on different sides of the robot
    // wheelBase - distance between pairs of wheels on the same side of the robot
    // THIS IS IMPORTANT FOR A RECTANGULAR ROBOT
    public static final double TRACKWIDTH = 1.0;
    public static final double WHEELBASE = 1.0;

    public static final int PRIMARY_CONTROLLER_PORT = 0;

    public static final int leftJoystickXAxis = 0;
    public static final int leftJoystickYAxis = 1;
    public static final int rightJoystickXAxis = 4;

    public static final double SPEED_DIVISOR = 3; // what the max speed should be divided by, 1 is max power
}
