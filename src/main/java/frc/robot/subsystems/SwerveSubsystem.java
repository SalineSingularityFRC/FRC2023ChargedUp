package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveClasses.SwerveModule;
import frc.robot.SwerveClasses.Vector;
import java.util.function.Consumer;
import java.util.function.Supplier;

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

  private final SwerveDriveKinematics swerveDriveKinematics;
  private ChassisSpeeds chassisSpeeds;
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
    gyro = new Pigeon2(Constants.CanId.CanCoder.GYRO, Constants.Canbus.DRIVE_TRAIN);

    vectorKinematics[FL] =
        new Vector(Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[FR] =
        new Vector(Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[BL] =
        new Vector(-Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[BR] =
        new Vector(
            -Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0);

    Translation2d[] wheel = new Translation2d[4];
    for (int i = 0; i < vectorKinematics.length; i++) {
      wheel[i] = new Translation2d(vectorKinematics[i].x, vectorKinematics[i].y);
    }

    swerveDriveKinematics = new SwerveDriveKinematics(wheel);

    chassisSpeeds = new ChassisSpeeds();
    swerveModules[FL] =
        new SwerveModule(
            Constants.CanId.Motor.FL,
            Constants.CanId.Angle.FL,
            Constants.CanId.CanCoder.FL,
            Constants.WheelOffset.FL,
            Constants.Canbus.DRIVE_TRAIN,
            Constants.Inverted.FL,
            "FL");
    swerveModules[FR] =
        new SwerveModule(
            Constants.CanId.Motor.FR,
            Constants.CanId.Angle.FR,
            Constants.CanId.CanCoder.FR,
            Constants.WheelOffset.FR,
            Constants.Canbus.DRIVE_TRAIN,
            Constants.Inverted.FR,
            "FR");
    swerveModules[BL] =
        new SwerveModule(
            Constants.CanId.Motor.BL,
            Constants.CanId.Angle.BL,
            Constants.CanId.CanCoder.BL,
            Constants.WheelOffset.BL,
            Constants.Canbus.DRIVE_TRAIN,
            Constants.Inverted.BL,
            "BL");
    swerveModules[BR] =
        new SwerveModule(
            Constants.CanId.Motor.BR,
            Constants.CanId.Angle.BR,
            Constants.CanId.CanCoder.BR,
            Constants.WheelOffset.BR,
            Constants.Canbus.DRIVE_TRAIN,
            Constants.Inverted.BR,
            "BR");

    Consumer<ChassisSpeeds> consumer_chasis =
        ch_speed -> {
          SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(ch_speed);
          setModuleStates(modules);
        };
    Supplier<ChassisSpeeds> supplier_chasis =
        () -> {
          SmartDashboard.putNumber("Chassis_PathPlanner_X", getChassisSpeed().vxMetersPerSecond);
          SmartDashboard.putNumber("Chassis_PathPlanner_Y", getChassisSpeed().vyMetersPerSecond);
          return getChassisSpeed(); // Maybe come back to this later
        };
    Supplier<Pose2d> supplier_position =
        () -> {
          SmartDashboard.putNumber("PathPlanner_Odometry_X", Robot.odometry.position().getX());
          SmartDashboard.putNumber("PathPlanner_Odometry_Y", Robot.odometry.position().getY());
          SmartDashboard.putNumber("PathPlanner_Odometry_Angle", Robot.odometry.position().getRotation().getRadians());
          return Robot.odometry.position(); // Maybe come back to this later
        };
    Consumer<Pose2d> consumer_position =
        pose -> {
          Robot.odometry.resetPosition(); // Maybe come back to this later
        };

    // SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(getChassisSpeed());
    // setModuleStates(modules);

    AutoBuilder.configureHolonomic(
        supplier_position, // Robot pose supplier
        consumer_position, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        supplier_chasis, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        consumer_chasis, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(.001, 0.0, 0.0), // Translation PID constants
            new PIDConstants(.001, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.70832, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Enum alliance = Alliance.Blue;
          //   // Boolean supplier that controls when the path will be mirrored for the red alliance
          //   // This will flip the path being followed to the red side of the field.
          //   // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          //   // var alliance = DriverStation.getAlliance();
          //   // if (alliance.isPresent()) {
          //   return alliance.get() == DriverStation.Alliance.Red;
          // }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
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

    // this is to make sure if both the joysticks are at neutral position, the robot
    // and wheels
    // don't move or turn at all
    // 0.05 value can be increased if the joystick is increasingly inaccurate at
    // neutral position
    if (Math.abs(swerveRequest.movement.x) <= 0.05
        && Math.abs(swerveRequest.movement.y) <= 0.05
        && Math.abs(swerveRequest.rotation) <= 0.05) {

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
        }
      } else {
        targetAngle = Double.MAX_VALUE;
      }
    }

    double x = swerveRequest.movement.x;
    double y = swerveRequest.movement.y;
    if (fieldCentric) {
      double difference = (startingAngle - currentRobotAngle) % (2 * Math.PI);
      x =
          -swerveRequest.movement.y * Math.sin(difference)
              + swerveRequest.movement.x * Math.cos(difference);
      y =
          swerveRequest.movement.y * Math.cos(difference)
              + swerveRequest.movement.x * Math.sin(difference);
    }

    this.chassisSpeeds = new ChassisSpeeds(y, x, swerveRequest.rotation);
    SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(modules);
  }

  public ChassisSpeeds getChassisSpeed() {
    return new ChassisSpeeds(Robot.odometry.getY(), Robot.odometry.getX(), Robot.odometry.position().getRotation().getRadians());
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
   * This function returns the angle (in radians) of the robot based on the value
   * from the pidgeon 2.0
   */
  public double getRobotAngle() {
    // return ((360 - gyro.getAngle().toDegrees()) * Math.PI) / 180; // for NavX
    return (((gyro.getAngle() - gyroZero)) * Math.PI)
        / 180; // returns in counterclockwise hence why 360 minus
    // it is gyro.getAngle() - 180 because the pigeon for this robot is facing
    // backwards
  }

  public void resetGyro() {
    // gyro.reset();
    gyroZero = gyro.getAngle();
    this.startingAngle = getRobotAngle() + Math.PI;
  }

  public Command resetGyroCommand() {
    return runOnce(() -> {resetGyro();});
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
