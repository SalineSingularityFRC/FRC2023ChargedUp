package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveRequest;

public class Limelight {
  private NetworkTable table;
  public NetworkTableEntry tx, ty, ta, tv, ledMode, camMode, pipeLine, crop;

  private double[] localization;
  private double poseX, poseY, yaw;

  public boolean isTurningDone;
  public final double minimumSpeed = 0.06;
  PIDController driveController;
  public PIDController turnController;
  PIDController scoreDriveController;

  public Timer scoringTimer = new Timer();
  public Timer pickupTimer = new Timer();

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    // swap the limelight between vision processing (0) and drive camera (1)
    camMode = table.getEntry("camMode");

    // state of LEDs: 1.0 - on, 2.0 - blink, 3.0 - off
    ledMode = table.getEntry("ledMode");

    pipeLine = table.getEntry("pipeline");

    localization = table.getEntry("botpose").getDoubleArray(new double[6]);
    poseX = localization[0];
    poseY = localization[1];
    yaw = localization[5];

    driveController = new PIDController(0.0025, 0, 0); // 0.0056 orginally
    driveController.setSetpoint(-18);

    // driveController.setTolerance(0.5);
    turnController = new PIDController(0.001, 0, 0.0008 / 8);
    turnController.setSetpoint(9);

    scoreDriveController = new PIDController(0.0056, 0, 0);
    scoreDriveController.setSetpoint(1.7); // FIND RIGHT TA VALUE

    // driveController.setTolerance(0.5);
    setCamMode(0); // set to vision mode
    ledOff();
    setpipeline(1);
  }

  public void runLimelight() { // to put smartdashboard values
    SmartDashboard.putNumber("tx", tx.getDouble(0.0));
    SmartDashboard.putNumber("ty", ty.getDouble(0.0));
    SmartDashboard.putNumber("ta", ta.getDouble(0.0));

    SmartDashboard.putNumber("poseX", poseX);
    SmartDashboard.putNumber("poseY", poseY);
    SmartDashboard.putNumber("yaw", yaw);
  }

  // turns on the LEDs
  public void ledOn() {
    ledMode.setDouble(3.0);
  }

  // turns off the LEDs
  public void ledOff() {
    ledMode.setDouble(0.0);
  }

  // method to switch camera between drive mode and vision mode
  public void setCamMode(double mode) {
    camMode.setDouble(mode);
  }

  // method to change between pipeLines, takes an int and a LimeLight object
  public void setpipeline(int pipe) {
    pipeLine.setNumber(pipe);
  }

  public boolean getIsTargetFound() {
    double a = ta.getDouble(0);
    if (a <= 0.05) {
      return false;
    } else {
      return true;
    }
  }

  public boolean pickup(
      SwerveSubsystem drive,
      ArmSubsystem arm,
      ClawPneumatics claw,
      LightSensor cubeLightSensor,
      LightSensor coneLightSensor,
      boolean isCube,
      boolean auton) {

    if (isCube) {
      setpipeline(2);
    } else {
      setpipeline(1); // is cone
    }

    arm.pickupTarget();
    ledOff();

    if (claw.isClawClosed) {
      claw.setLow();
    }

    if (isCube) {
      if (!auton) {
        if (driveController.atSetpoint()) {
          claw.setHigh();
          return true;
        }
      } else {
        if (cubeLightSensor.isSensed()) {
          pickupTimer.start();

          double speed = turnController.calculate(tx.getDouble(0));
          drive.drive(new SwerveSubsystem.SwerveRequest(-speed, 0, 0.1), false);

          if (pickupTimer.get() >= 0.02) {
            drive.drive(new SwerveRequest(0, 0, 0.1), false);
            return true;
          }
        }
      }
    } else {
      if (coneLightSensor.isSensed()) { // we really want the sensor for this tbh
        claw.setHigh();
        return true;
      }
    }

    if (tx.getDouble(0) < 6.5 || tx.getDouble(0) > 11.5) {
      isTurningDone = false;
    }
    if (!isTurningDone) {
      isTurningDone = turnAngle(drive, true);
    } else {

      double y = driveController.calculate(ty.getDouble(0));
      if (y > 0) y += 0.06;
      if (y < 0) y -= 0.06;
      SmartDashboard.putNumber("Calculated Y value", y);
      // We see the target and are aimed at it, drive forwards now

      // if (ty.getDouble(0) >= -18) {
      //     // 0.26 is a specific multiplier that makes speed hit exactly 0.7 if ta is approx 0
      // (very far away)
      //     y = ((18 + ty.getDouble(0)) * 0.013) + minimumSpeed;
      //     if(y > 0.35){
      //         y = 0.35;
      //     }

      //     SmartDashboard.putNumber("y value", y);
      // }

      // if (tx.getDouble(0) >= 9.25) {
      //     x = ((tx.getDouble(0) - 9.25) * 0.005) + minimumSpeed;
      //     if (x < -0.1) {
      //         x = -0.1;
      //     }

      //     x *= -1;
      // }
      // else if (tx.getDouble(0) < 8.75) {
      //     x = (8.75 - (tx.getDouble(0)) * 0.005) + minimumSpeed;
      //     if (x > 0.1) {
      //         x = 0.1;
      //     }
      // }

      if (this.pickupTimer.get() == 0) {
        double speed = turnController.calculate(tx.getDouble(0));
        drive.drive(new SwerveRequest(-speed, 0, -y * 2.5), false);
      }
    }

    return false;
  }

  public boolean turnAngle(SwerveSubsystem drive, boolean pickup) {
    if (getIsTargetFound()) {

      double speed = turnController.calculate(tx.getDouble(0));
      if (speed > 0) speed += 0.05;
      if (speed < 0) speed -= 0.05;
      // if (tx.getDouble(0) >= 9.5) {
      //     // 0.005 is a specific multiplier that makes speed hit exactly 0.2 if bot is off by 20
      // degrees to the left
      //     //speed = ((tx.getDouble(0) - 9.5) * 0.005) + minimumSpeed;
      //     if(speed > 0.2){
      //         speed = 0.2;
      //     }

      // }
      // else if (tx.getDouble(0) <= 8.5) {
      //     // 0.01 is a specific multiplier that makes speed hit exactly -0.2 if bot is off by 20
      // degrees to the right
      //     speed = ((8.5 - tx.getDouble(0)) * 0.01) + minimumSpeed;
      //     if(speed > 0.2){
      //         speed = 0.2;
      //     }

      //     speed *= -1; // because we want to turn clockwise here
      // }
      if (turnController.atSetpoint()) {
        turnController.reset();
        // drive.drive(new SwerveRequest(0 , 0, 0), false);
        return true; // we are angled correctly
      }
      if (pickup) {
        if (pickupTimer.get() == 0) drive.drive(new SwerveRequest(-speed, 0, 0), false);
      } else {
        drive.drive(new SwerveRequest(-speed, 0, 0), false);
      }
    }

    return false;
  }

  public boolean score(
      SwerveSubsystem drive, ArmSubsystem arm, ClawPneumatics claw, boolean isCube) {
    // arm.defaultTarget();
    ledOn();
    if (isCube) {
      setpipeline(0);
    } else {
      setpipeline(3);
    }
    if (tx.getDouble(0) < 6.5 || tx.getDouble(0) > 11.5) {
      isTurningDone = false;
    }
    if (!isTurningDone) {
      isTurningDone = turnAngle(drive, false);
    } else {
      if (scoreDriveController.atSetpoint()) {
        claw.setLow();
        return true;
      }

      double y = scoreDriveController.calculate(ta.getDouble(0));
      if (y > 0) y += 0.1;
      if (y < 0) y -= 0.1;

      double speed = turnController.calculate(tx.getDouble(0));
      drive.drive(new SwerveRequest(-speed, 0, y * 2), false);
    }
    // double robotAngle = (drive.getRobotAngle() % (Math.PI * 2)) * (180/Math.PI); // in angles
    // double x = 0;
    // double y = 0;

    // // 0.00061 is a specific multipler to make the rotation peak at 0.15 (including min speed)
    // double rotation = (180 - robotAngle) * 0.00061;
    // if (rotation >= 0) {
    //     rotation += minimumSpeed;
    // }
    // else {
    //     rotation -= minimumSpeed;
    // }

    // if (ty.getDouble(0) >= 1.5) {
    //     // 0.016 is a specific multiplier that makes speed hit exactly 0.3 if ty is approx 16.5
    //     y = ((ty.getDouble(0) - 1.5) * 0.016) + minimumSpeed;
    //     if(y > 0.3){
    //         y = 0.30;
    //     }
    // }

    // if (tx.getDouble(0) >= 14.5) {
    //     x = ((tx.getDouble(0) - 14.5) * 0.0055) + minimumSpeed;
    //     if (x < -0.15) {
    //         x = -0.15;
    //     }

    //     x *= -1;
    // }
    // else if (tx.getDouble(0) < 13.5) {
    //     x = (13.5 - (tx.getDouble(0)) * 0.0055) + minimumSpeed;
    //     if (x > 0.15) {
    //         x = 0.15;
    //     }
    // }

    // if (tx.getDouble(0) <= 13.5 && tx.getDouble(0) >= 14.5 && ty.getDouble(0) <= 1.5
    //                 && robotAngle <= 181 && robotAngle >= 179) {
    //     scoringTimer.start();
    //     arm.highTarget(scoringTimer);
    //     if (scoringTimer.get() >= 1)  {
    //         claw.setLow();
    //         return true;
    //     }
    // }
    // else {
    //     drive.drive(new SwerveRequest(rotation, x, y), true); // we want field centric here
    // }

    return false;
  }

  public void turnToAngle(SwerveSubsystem drive) {
    double robotAngle = (drive.getRobotAngle() % (Math.PI * 2)) * (180 / Math.PI); // in degrees
    SmartDashboard.putNumber("robotAngle", robotAngle);
    double rotation = (180 - robotAngle) * 0.0061;
    if (rotation >= minimumSpeed) {
      rotation -= minimumSpeed;
    } else if (rotation <= -minimumSpeed) {
      rotation += minimumSpeed;
    } else {
      rotation = 0;
      return;
    }

    drive.drive(new SwerveRequest(rotation, 0, 0), false);
  }
}
