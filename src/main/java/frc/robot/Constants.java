package frc.robot;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {
    public static final double ANGLE_MOTOR_GEAR_RATIO = 8.14; // Yuri: "If its not 8.14 its 6.75"
    public static final double MAX_ANGLE_INACCURACY = Math.PI/ 24;
    public static final String CANBUS = "rio";

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

    public static final int FL_ANGLE_ID = 1; 
    public static final int FL_Motor_ID = 2;

    public static final int FR_ANGLE_ID = 3; 
    public static final int FR_Motor_ID = 4;

    public static final int BL_ANGLE_ID = 5; 
    public static final int BL_Motor_ID = 6;

    public static final int BR_ANGLE_ID = 7; 
    public static final int BR_Motor_ID = 8;
}
