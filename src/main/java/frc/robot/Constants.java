package frc.robot;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {
    public static final double ANGLE_MOTOR_GEAR_RATIO = 6.45; // Yuri: "If its not 8.14 its 6.75"
    public static final double MAX_ANGLE_INACCURACY = Math.PI/ 24;
    public static final String CANBUS = "rio";
    public static final String CANIVORE = "drivetrain";


    /* 
    PLACEHOLDER WIDTH AND LENGTH VALUES
    These values are in inches
    Width is from FL to FR, length is from FL to BL etc
    Width and length are for the distance between each swervepod
    Length will be longer than width for 2023 robot
    */
    public static final double robotWidth = 28; 
    public static final double robotLength = 32;

    public static final double theta = Math.atan(robotWidth/robotLength); 

    
    // --------->  THESE ARE JUST PLACEHOLDER ID VALUES <------------
    public static final int GYRO_CANCODER_ID = 0; 

    public static final int FL_ANGLE_ID = 17; 
    public static final int FL_Motor_ID = 15;

    public static final int FR_ANGLE_ID = 12; 
    public static final int FR_Motor_ID = 16;

    public static final int BL_ANGLE_ID = 13; 
    public static final int BL_Motor_ID = 11;

    public static final int BR_ANGLE_ID = 14; 
    public static final int BR_Motor_ID = 18;
  


     public static double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(0);
    public static double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(0);
    public static double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Math.toRadians(0);
    public static double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(0);
}
