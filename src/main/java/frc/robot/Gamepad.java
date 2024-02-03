package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

/** Main class to control the robot */
public class Gamepad {

  public String allianceColor = DriverStation.getAlliance().toString();

  private Timer highTargetTimer = new Timer();
  private Timer sliderTimer = new Timer();
  private Timer defaultTimer = new Timer();
  private Timer pickupFallenConeTimer = new Timer();
  private Timer clawCloseTimer = new Timer();
  private Timer clawOpenTimer = new Timer();

  //private Joystick driveController;
  //private Joystick armController;

  /**
   * @param driveControllerPort Controller port the drive controller is connected to, probably 0
   * @param armControllerPort Controller port the arm controller is connect to, probably 1
   */
  public Gamepad(int driveControllerPort, int armControllerPort) {
    //driveController = new Joystick(driveControllerPort);
    //armController = new Joystick(armControllerPort);
  }

  public void armPneumatics(
      ClawPneumatics clawPneumatics,
      LightSensor coneLightSensor,
      LightSensor cubeLightSensor,
      ArmSubsystem arm) {

    if (clawCloseTimer.get() >= 0.25) {
      if ((arm.smallArmMotorPosition + (Constants.Speed.ARM * 4.5))
          < Constants.Position.SmallArm.DEFAULT) {
        arm.smallArmMotorPosition += Constants.Speed.ARM * 4.5;
        arm.maintainPosition();
      }
      clawCloseTimer.stop();
      clawCloseTimer.reset();
    }

    if (clawOpenTimer.get() >= 0.25) {
      clawPneumatics.setLow();
      clawOpenTimer.stop();
      clawOpenTimer.reset();
    }
    /*
    if (cubeLightSensor.isSensed()
        && (armController.getRawButton(Constants.Gamepad.Button.RIGHT))) {
      clawPneumatics.setHigh();
      clawCloseTimer.start();

    } else if (!coneLightSensor.isSensed()
        && (armController.getRawButton(Constants.Gamepad.Button.BACK))) {
      clawPneumatics.setHigh();
      clawCloseTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.Gamepad.Button.A)) {
      if (clawPneumatics.isClawClosed) {
        if (arm.bigArmMotorPosition > Constants.Position.BigArm.PICKUP) {
          arm.smallArmMotorPosition -= Constants.Speed.ARM * 9;
        }
        arm.maintainPosition();
        clawOpenTimer.start();
      } else {
        clawPneumatics.setHigh();
        clawCloseTimer.start();
      }
    } else if (driveController.getRawButtonReleased(Constants.Gamepad.Button.A)) {
      clawPneumatics.setOff();
    }

    if (driveController.getRawButtonPressed(Constants.Gamepad.Button.B)) {
      clawPneumatics.toggleCompressor();
    }
    */
  }

  public void swerveDrive(
      SwerveSubsystem robotSubsystem,
      Limelight limelight,
      ArmSubsystem arm,
      ClawPneumatics claw,
      LightSensor cubeLightSensor,
      LightSensor coneLightSensor,
      SwerveOdometry odometry) {

    if (limelight.pickupTimer.get() >= 0.9) {

      robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(0, 0, 0), true);
      claw.setHigh();

      limelight.pickupTimer.stop();
      limelight.pickupTimer.reset();
    }
    // limelight commands below
    /*
    if (armController.getPOV() == 0) {
      limelight.turnToAngle(robotSubsystem);
    }

    if (armController.getRawButtonPressed(Constants.Gamepad.Button.L_JOYSTICK)
        || armController.getRawButtonPressed(Constants.Gamepad.Button.R_JOYSTICK)) {
      claw.setLow();
      limelight.turnController.setP(0.0025);
      limelight.isTurningDone = false;
      limelight.pickupTimer.stop(); // just in case
      limelight.pickupTimer.reset();
      limelight.scoringTimer.stop(); // just in case
      limelight.scoringTimer.reset();
      robotSubsystem.setBrakeMode();
    }

    if (armController.getRawButtonReleased(Constants.Gamepad.Button.L_JOYSTICK)
        || armController.getRawButtonReleased(Constants.Gamepad.Button.R_JOYSTICK)) {
      robotSubsystem.setCoastMode();
    }

    if (armController.getRawButton(Constants.Gamepad.Button.L_JOYSTICK) && !claw.isClawClosed) {
      limelight.pickup(robotSubsystem, arm, claw, cubeLightSensor, coneLightSensor, false, false);
    } else if (armController.getRawButton(Constants.Gamepad.Button.R_JOYSTICK)
        && !claw.isClawClosed) {
      limelight.pickup(robotSubsystem, arm, claw, cubeLightSensor, coneLightSensor, true, true);
    } else { // no limelight commands`
      if (driveController.getRawButtonPressed(Constants.Gamepad.Button.X)) {
        robotSubsystem.resetGyro();
      }

      if (armController.getRawButton(Constants.Gamepad.Button.LEFT)) {
        if (robotSubsystem.isCoast()) {
          robotSubsystem.setBrakeMode();
        } else {
          robotSubsystem.setCoastMode();
        }
      }
      robotSubsystem.drive(
          new SwerveSubsystem.SwerveRequest(
              driveController.getRawAxis(Constants.Gamepad.Axis.RIGHT),
              -driveController.getRawAxis(Constants.Gamepad.Axis.LEFT_X),
              -driveController.getRawAxis(Constants.Gamepad.Axis.LEFT_Y)),
          true);
    }
    */
  }
}
