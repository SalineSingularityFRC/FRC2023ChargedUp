package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.SwerveClasses.ChassisVelocity;
import frc.robot.SwerveClasses.SwerveDriveRequest;
import frc.robot.SwerveClasses.SwerveKinematics;
import frc.robot.SwerveClasses.SwerveModule;
import frc.robot.SwerveClasses.Vector;

// import com.kauailabs.navx.frc.AHRS;
// import frc.robot.DumbNavXClasses.NavX;

/*
 * This class provides functions to drive at a given angle and direction,
 * and performs the calculations required to achieve that
 */
public class SwerveSubsystem implements Subsystem {
  // public class SwerveSubsystem implements UpdateManager.Updatable {
  /*
   * This class should own the pidgeon 2.0 IMU gyroscope that we will be using and
   * a dictionary that will house
   * all of our SwerveModules
   */
  public Pigeon2 gyro;

  private final int FL = 0;
  private final int FR = 1;
  private final int BL = 2;
  private final int BR = 3;

  public SwerveModule[] swerveModules = new SwerveModule[4];
  public final Vector[] vectorKinematics = new Vector[4];
  private final SwerveKinematics swerveKinematics;

  public double gyroZero = 0;

  private double targetAngle = Double.MAX_VALUE;

  private double startingAngle;

  /*
   * This constructor should create an instance of the pidgeon class, and should
   * construct four copies of the
   * SwerveModule class and add them to our SwerveModule dictionary
   * Use values from the Constants.java class
   */
  public SwerveSubsystem() {
    // gyro = new NavX(Port.kMXP);
    gyro = new Pigeon2(Constants.CanId.CanCoderID.GYRO, Constants.Canbus.DEFAULT);

    vectorKinematics[FL] =
        new Vector(Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[FR] =
        new Vector(Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[BL] =
        new Vector(-Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[BR] =
        new Vector(-Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0);

    swerveKinematics = new SwerveKinematics(vectorKinematics);

    swerveModules[FL] =
        new SwerveModule(
            Constants.CanId.MotorID.FL,
            Constants.CanId.AngleID.FL,
            Constants.CanId.CanCoderID.FL,
            Constants.WheelOffset.FL_ENCODER,
            Constants.Canbus.DEFAULT,
            Constants.Inverted.FL,
            "FL");
    swerveModules[FR] =
        new SwerveModule(
            Constants.MotorID.FR,
            Constants.AngleID.FR,
            Constants.CanCoderID.FR,
            Constants.WheelOffset.FRONT_RIGHT_ENCODER,
            Constants.Canivore.CANIVORE,
            Constants.isInverted.FR,
            "FR");
    swerveModules[BL] =
        new SwerveModule(
            Constants.MotorID.BL,
            Constants.AngleID.BL,
            Constants.CanCoderID.BL,
            Constants.WheelOffset.BACK_LEFT_ENCODER,
            Constants.Canivore.CANIVORE,
            Constants.isInverted.BL,
            "BL");
    swerveModules[BR] =
        new SwerveModule(
            Constants.MotorID.BR,
            Constants.AngleID.BR,
            Constants.CanCoderID.BR,
            Constants.WheelOffset.BACK_RIGHT_ENCODER,
            Constants.Canivore.CANIVORE,
            Constants.isInverted.BR,
            "BR");
  }

  public static
  class SwerveRequest { // this class represents what our controller is telling our robot to do
    public double rotation;
    public Vector movement;

    public SwerveRequest(double rotation, double x, double y) {
      this.rotation = rotation;
      this.movement = new Vector(x, y);
    }
  }

  public void drive(
      SwerveRequest swerveRequest,
      boolean fieldCentric) { // takes in the inputs from the controller
    double currentRobotAngle = getRobotAngle();
    ChassisVelocity chassisVelocity;

    SmartDashboard.putNumber("x", swerveRequest.movement.x);
    SmartDashboard.putNumber("y", swerveRequest.movement.y);

    // this is to make sure if both the joysticks are at neutral position, the robot
    // and wheels
    // don't move or turn at all
    // 0.05 value can be increased if the joystick is increasingly inaccurate at
    // neutral position
    if (Math.abs(swerveRequest.movement.x) < 0.05
        && Math.abs(swerveRequest.movement.y) < 0.05
        && Math.abs(swerveRequest.rotation) < 0.05) {

      targetAngle = Double.MAX_VALUE;

      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].coast();
      }
      return;
    } else {

      // this is to drive straight
      if (Math.abs(swerveRequest.rotation) < 0.05) {
        if (targetAngle == Double.MAX_VALUE) {
          targetAngle = getRobotAngle();
        } else {
          double difference = getRobotAngle() - targetAngle;
          swerveRequest.rotation = difference;
          SmartDashboard.putNumber("Difference", difference);
        }
      } else {
        targetAngle = Double.MAX_VALUE;
      }

      SmartDashboard.putNumber("ROTATION", swerveRequest.rotation);
      SmartDashboard.putNumber("TARGET ANGLE", targetAngle);
      SmartDashboard.putNumber("GET ROBOT ANGLE", getRobotAngle());
    }

    double x = swerveRequest.movement.x;
    double y = swerveRequest.movement.y;

    /*
     * This is to change the vector value from robo centric to field centric
     * The code is used to calculate the x and y components of the joystick input in
     * the field-centric coordinate system.
     * The difference variable is the difference between the robot’s orientation and
     * the field’s orientation.
     * The code uses Math.sin() and Math.cos() functions to calculate the x and y
     * components of the joystick input in the field-centric coordinate system
     */
    if (fieldCentric) {
      double difference = (currentRobotAngle - startingAngle) % (2 * Math.PI);
      SmartDashboard.putNumber("DIFFERENCE FIELD CENTRIC", difference);
      x =
          -swerveRequest.movement.y * Math.sin(difference)
              + swerveRequest.movement.x * Math.cos(difference);
      y =
          swerveRequest.movement.y * Math.cos(difference)
              + swerveRequest.movement.x * Math.sin(difference);
    }

    chassisVelocity = new ChassisVelocity(new Vector(x, y), swerveRequest.rotation);

    Vector[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
    SmartDashboard.putNumber("MODULE OUTPUTS WHELL IDK X", moduleOutputs[0].x);
    SmartDashboard.putNumber("MODULE OUTPUES WHELL IDK Y", moduleOutputs[0].y);
    SwerveKinematics.normalizeModuleVelocities(
        moduleOutputs, 1); // these two lines are what calculates the module angles for swerve

    for (int i = 0; i < moduleOutputs.length; i++) {
      SwerveModule module = swerveModules[i];
      SwerveDriveRequest request;

      // String string = "wheel #" + i;

      // These if else are to swap the moduleOutputs for the FR and BL modules
      // TO-DO research on why we need this, but for now, it works as it is so its
      // fine
      if (i == 1) {
        i = 2;
        request = driveInstructions(moduleOutputs[i]);
        i = 1;
      } else if (i == 2) {
        i = 1;
        request = driveInstructions(moduleOutputs[i]);
        i = 2;
      } else {
        request = driveInstructions(moduleOutputs[i]);
        /*
         * this method is used to convert the vector that the swerveKinematics outputs
         * for each wheel
         * into direction and speed instructions that the module.drive method can take
         * in
         */
      }

      if (request.direction != 0) {
        request.direction = (2 * Math.PI) - request.direction;
        // IMPORTANT: this bit is to convert the direction from clockwise to
        // counterclockwise
        // Their kinematics class outputs clockwise degree, but our methods take in a
        // counterclockwise degree
      }

      module.drive(request);
    }
  }

  /*
   * Odometry
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // The 2nd Parameter is for MaxSpeedMetersPerSecond
    // Initial Value was 3
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 3);
    swerveModules[FL].setDesiredState(desiredStates[0]);
    swerveModules[FR].setDesiredState(desiredStates[1]);
    swerveModules[BL].setDesiredState(desiredStates[2]);
    swerveModules[BR].setDesiredState(desiredStates[3]);
  }

  public void setModuleState(SwerveModuleState desiredStates) {
    // The 2nd Parameter is for MaxSpeedMetersPerSecond
    // Initial Value was 3
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 3);
    swerveModules[FL].setDesiredState(desiredStates);
    swerveModules[FR].setDesiredState(desiredStates);
    swerveModules[BL].setDesiredState(desiredStates);
    swerveModules[BR].setDesiredState(desiredStates);
  }

  /*
   * This method takes a field-centric
   * direction vector for
   * which way the module should travel and outputs the SwerveDriveRequest
   * instruction for the individual module
   */
  public SwerveDriveRequest driveInstructions(Vector vector) {
    double x = vector.x;
    double y = vector.y;

    double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) / Constants.Speed.ROBOT_SPEED_DIVISOR;
    double angle;

    if (y == 0) { // y = 0 wouldn't work because fraction
      if (x > 0) {
        angle = (3 * Math.PI) / 2;
      } else if (x < 0) {
        angle = Math.PI / 2;
      } else { // x = 0
        return new SwerveDriveRequest(0, 0); // this should never happen
      }
    } else if (y < 0 || (x == 0 && y < 0)) { // Q3 and Q4 and south field centric
      angle = Math.PI - Math.atan(x / y);
    } else if (x > 0 && y > 0) { // Q1 or north field centric
      angle = 2 * Math.PI - Math.atan(x / y);
    } else if (x <= 0 && y > 0) { // Q2 or north field centric
      angle = -1 * Math.atan(x / y);
    } else { // this else statement is useful as a catch all
      angle = 0;
    }

    return new SwerveDriveRequest(speed, angle);
  }

  /*
   * This function returns the angle (in radians) of the robot based on the value
   * from the pidgeon 2.0
   */
  public double getRobotAngle() {
    // return ((360 - gyro.getAngle().toDegrees()) * Math.PI) / 180; // for NavX
    return ((180 - (gyro.getAngle() - gyroZero)) * Math.PI)
        / 180; // returns in counterclockwise hence why 360 minus
    // it is gyro.getAngle() - 180 because the pigeon for this robot is facing
    // backwards
  }

  public void resetGyro() {
    // gyro.reset();
    gyroZero = gyro.getAngle();
    this.startingAngle = getRobotAngle() + Math.PI;
  }

  public SwerveModule getSwerveModule(int module) {
    return swerveModules[module];
  }

  public void setBrakeMode() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setBrakeMode();
    }
  }

  public void setCoastMode() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setCoastMode();
    }
  }

  public boolean isCoast() {
    return swerveModules[0].isCoast();
  }
}
