package frc.robot;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {
  /*
  ALL PHYSICAL IDS HERE
  */
  public static final int GYRO_CANCODER_ID = 0;

  public static final int FL_ANGLE_ID = 15;
  public static final int FL_Motor_ID = 17;
  public static final int FL_CANCODER_ID = 43;
  public static final boolean FL_isInverted = false;

  public static final int FR_ANGLE_ID = 16;
  public static final int FR_Motor_ID = 12;
  public static final int FR_CANCODER_ID = 44;
  public static final boolean FR_isInverted = true;

  public static final int BL_ANGLE_ID = 11;
  public static final int BL_Motor_ID = 13;
  public static final int BL_CANCODER_ID = 42;
  public static final boolean BL_isInverted = false;

  public static final int BR_ANGLE_ID = 18;
  public static final int BR_Motor_ID = 14;
  public static final int BR_CANCODER_ID = 41;
  public static final boolean BR_isInverted =
      true; // Changed this 2/18/23 due to wheels not rotating right way

  public static final int BIG_ARM_Motor_ID = 6;
  public static final int BIG_ARM_Motor_2_ID = 7;
  public static final int SMALL_ARM_MOTOR_ID = 31;

  public static final int Compressor_ID = 1;

  public static final int SMALL_ARM_CANCODER_ID = 22;
  public static final int BIG_ARM_CANCODER_ID = 21;

  public static final String CANBUS = "rio";
  public static final String CANIVORE = "drivetrain";

  public static final int CUBE_SENSOR_CHANNEL = 2;
  public static final int CONE_SENSOR_CHANNEL = 3;

  /*
  GAMEPAD IDS BELOW
  */
  public static final int DRIVE_CONTROLLER = 0;
  public static final int ARM_CONTROLLER = 1;

  public static final int leftJoystickXAxis = 0;
  public static final int leftJoystickYAxis = 1;
  public static final int leftTrigger = 2;
  public static final int rightTrigger = 3;
  public static final int rightJoystickXAxis = 4;

  public static final int A_Button = 1;
  public static final int B_Button = 2;
  public static final int X_Button = 3;
  public static final int Y_Button = 4;
  public static final int left_Button = 5;
  public static final int right_Button = 6;
  public static final int Back_Button = 7;
  public static final int Start_Button = 8;
  public static final int L_joystick_Button = 9;
  public static final int R_joystick_Button = 10;

  /*
  ALL WHEEL OFFSETS BELOW
  */
  public static double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET =
      (0.811035) * 2 * Math.PI; // number in parenthesis is in rotations
  public static double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = (0.487061) * 2 * Math.PI; // 0.884521
  public static double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = (0.219971) * 2 * Math.PI; //  0.063965
  public static double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = (0.534180) * 2 * Math.PI; // 0.811768

  /*
  ALL MOTOR GEAR RATIOS BELOW
  */
  public static final double DRIVE_MOTOR_GEAR_RATIO = 8.14; // this is the ratio of bot
  public static final double ANGLE_MOTOR_GEAR_RATIO =
      12.8; // https://www.swervedrivespecialties.com/products/mk4-swerve-module

  public static final int BIG_ARM_GEAR_RATIO = 10;
  public static final int SMALL_ARM_GEAR_RATIO = 7;

  /*
  MISC. NUMBERS BELOW
  */
  public static final double MAX_ANGLE_INACCURACY = Math.PI / 24;

  public static final double BigArm_highTarget = 0.578857;
  public static final double BigArm_mediumTarget = 0.437500;
  public static final double BigArm_pickup = 0.392578;
  public static final double BigArm_default = 0.386475;
  public static final double BigArm_slider = 0.569092; // Fine tune
  public static final double BigArm_pickupCone = 0.477295;

  public static final double SmallArm_highTarget = -0.105736;
  public static final double SmallArm_mediumTarget = 0.164062;
  public static final double SmallArm_pickup = -0.014551;
  public static final double SmallArm_default = 0.322266;
  public static final double SmallArm_Slider = -0.203028;
  public static final double SmallArm_pickupCone = -0.272852;

  // 15.479004 position for bigarm for smallarm to freely rotate
  // 42.833496 highest position for big arm

  // trackWidth - lateral distance between pairs of wheels on different sides of the robot
  // wheelBase - distance between pairs of wheels on the same side of the robot
  // THIS IS IMPORTANT FOR A RECTANGULAR ROBOT
  public static final double TRACKWIDTH = 0.85;
  public static final double WHEELBASE =
      1.1333; // the ratio between the width and the length is around 3:4
  // public static final double TRACKWIDTH = 1;
  // public static final double WHEELBASE = 1;

  public static final double SPEED_DIVISOR =
      1; // what the max speed should be divided by, 1 is max power
  public static final double ARM_SPEED = 0.005; // speed of the arms when adjusting manually

  // CHARGE STATION COMMUNITY DISTANCE:
  public static final double encoderToChargeDistance = 96.4694981;

  // public static final double encoderToChargeDistance = 45.4694981;

  public static final double encoderCenterCommunity = 100;
  public static final double encoderToOutsideCommunityDistance = 87.30208217;
  // 1.832716884 is the number of inches per 1 encoder value
  // ~80 (plus offset) to the center of the charge station for robot
  // ~160 is the distance to leave the community plus some extra cushion
}
