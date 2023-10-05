package frc.robot;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {

  public static final class CanCoderID {
    public static final int GYRO = 0;
    public static final int FL = 43;
    public static final int FR = 44;
    public static final int BL = 42;
    public static final int BR = 41;
    public static final int SMALL_ARM = 22;
    public static final int BIG_ARM = 21;
  }

  public static final class AngleID {
    public static final int FL = 15;
    public static final int FR = 16;
    public static final int BL = 11;
    public static final int BR = 18;
  }

  public static final class MotorID {
    public static final int FL = 17;
    public static final int FR = 12;
    public static final int BL = 13;
    public static final int BR = 14;
    public static final int BIG_ARM = 6;
    public static final int BIG_ARM_2 = 7;
    public static final int SMALL_ARM = 31;
    public static final int Compressor = 1;
  }

  public static final class BigArmPosition {
    public static final double HIGH = 0.578857;
    public static final double MEDIUM = 0.437500;
    public static final double PICKUP = 0.392578;
    public static final double DEFAULT = 0.386475;
    public static final double SLIDER = 0.595092; // Fine tune
    public static final double PICKUPCONE = 0.47295;
  }

  public static final class isInverted {
    // This is for moters
    public static final boolean FL = false;
    public static final boolean FR = true;
    public static final boolean BL = false;
    public static final boolean BR = true;
  }

  public static final class Canbus {
    public static final String CANBUS = "rio";
  }

  public static final class Canivore {
    public static final String CANIVORE = "drivetrain";
  }

  public static final class Sensor {
    public static final String CANIVORE = "drivetrain";
    public static final int CUBE_CHANNEL = 2;
    public static final int CONE_CHANNEL = 3;
  }

  public static final class Gamepad {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int ARM_CONTROLLER = 1;

    public static final int LEFT_JOYSTICK_XAXIS = 0;
    public static final int LEFT_JOYSTICK_YAXIS = 1;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int RIGHT_JOYSTICK_XAXIS = 4;

    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUTTON = 5;
    public static final int RIGHT_BUTTON = 6;
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
    public static final int L_JOYSTICK_BUTTON = 9;
    public static final int R_JOYSTICK_BUTTON = 10;
  }

  public static final class WheelOffset {
    // Converting rotations to radians
    public static final double FRONT_LEFT_ENCODER = (0.793945) * 2 * Math.PI;
    public static final double FRONT_RIGHT_ENCODER = (0.489502) * 2 * Math.PI;
    public static final double BACK_LEFT_ENCODER = (0.234375) * 2 * Math.PI;
    public static final double BACK_RIGHT_ENCODER = (0.520752) * 2 * Math.PI;
  }

  public static final class MotorGearRatio {
    public static final double DRIVE = 8.14; // this is the ratio of bot
    public static final double ANGLE =
        12.8; // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    public static final int BIG = 10;
    public static final int SMALL = 7;
  }

  public static final class SmallArmPosition {
    public static final double HIGH = -0.105736;
    public static final double MEDIUM = 0.164062;
    public static final double PICKUP = -0.014551;
    public static final double DEFAULT = 0.322266;
    public static final double SLIDER = -0.18003028;
    public static final double PICKUP_CONE = -0.272852;
  }

  public static final class Measurement {
    // trackWidth - lateral distance between pairs of wheels on different sides of
    // the robot
    // wheelBase - distance between pairs of wheels on the same side of the robot
    // THIS IS IMPORTANT FOR A RECTANGULAR ROBOT
    public static final double TRACKWIDTH = 0.85;
    public static final double WHEELBASE =
        1.1333; // the ratio between the width and the length is around 3:4
  }

  public static final class Speed {
    public static final double ROBOT_SPEED_DIVISOR =
        1; // what the max speed should be divided by, 1 is max power
    public static final double ARM_SPEED = 0.005; // speed of the arms when adjusting manually
  }

  public static final class Distance {
    // CHARGE STATION COMMUNITY DISTANCE:
    public static final double TO_BLUE_CHARGE_STATION = 96.4694981;
    public static final double TO_RED_CHARGE_STATION = 99;
    public static final double TO_CENTER_COMMUNITY = 100;
    public static final double TO_OUTSIDE_COMMUNITY = 87.30208217;
    // 1.832716884 is the number of inches per 1 encoder value
    // ~80 (plus offset) to the center of the charge station for robot
    // ~160 is the distance to leave the community plus some extra cushion
  }

  public static final class AngleInaccuracy {
    public static final double MAX = Math.PI / 24;
  }
}
