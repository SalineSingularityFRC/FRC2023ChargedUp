package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
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

  private Joystick driveController;
  private Joystick armController;

  // private boolean isConstantMode;

  /**
   * @param driveControllerPort Controller port the drive controller is connected to, probably 0
   * @param armControllerPort Controller port the arm controller is connect to, probably 1
   */
  public Gamepad(int driveControllerPort, int armControllerPort) {
    driveController = new Joystick(driveControllerPort);
    armController = new Joystick(armControllerPort);
  }

  public void driveConstant(SwerveSubsystem robotSubsystem) {
    SwerveModuleState desiredState = new SwerveModuleState();
    desiredState.angle = new Rotation2d();
    desiredState.speedMetersPerSecond = 1;
    robotSubsystem.setModuleState(desiredState);
  }

  public void armPneumatics(
      ClawPneumatics clawPneumatics,
      LightSensor coneLightSensor,
      LightSensor cubeLightSensor,
      ArmSubsystem arm) {
    // SmartDashboard.putBoolean("if True then not full yet", clawPneumatics.isNotFull());

    // if(armController.getRawButtonPressed(Constants.right_Button)) {
    //     candle.turnOn();
    // }
    // if(armController.getRawButtonReleased(Constants.right_Button)) {
    //     candle.turnOff();
    // }
    if (clawCloseTimer.get() >= 0.25) {
      if ((arm.smallArmMotorPosition + (Constants.Speed.ARM_SPEED * 4.5))
          < Constants.SmallArmPosition.DEFAULT) {
        arm.smallArmMotorPosition += Constants.Speed.ARM_SPEED * 4.5;
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
    SmartDashboard.putBoolean("Cube Light Sensor is Sensed", cubeLightSensor.isSensed());
    SmartDashboard.putBoolean("Cone Light Sensor is Sensed", !coneLightSensor.isSensed());
    if (cubeLightSensor.isSensed()
        && (armController.getRawButton(Constants.Gamepad.RIGHT_BUTTON))) {
      clawPneumatics.setHigh();
      clawCloseTimer.start();

    } else if (!coneLightSensor.isSensed()
        && (armController.getRawButton(Constants.Gamepad.BACK_BUTTON))) {
      clawPneumatics.setHigh();
      clawCloseTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.Gamepad.A_BUTTON)) {
      if (clawPneumatics.isClawClosed) {
        if (arm.bigArmMotorPosition > Constants.BigArmPosition.PICKUP) {
          arm.smallArmMotorPosition -= Constants.Speed.ARM_SPEED * 9;
        }
        arm.maintainPosition();
        clawOpenTimer.start();
      } else {
        clawPneumatics.setHigh();
        clawCloseTimer.start();
      }
    } else if (driveController.getRawButtonReleased(Constants.Gamepad.A_BUTTON)) {
      clawPneumatics.setOff();
    }

    if (driveController.getRawButtonPressed(Constants.Gamepad.B_BUTTON)) {
      clawPneumatics.toggleCompressor();
    }
  }

  public void swerveDrive(
      SwerveSubsystem robotSubsystem,
      Limelight limelight,
      ArmSubsystem arm,
      ClawPneumatics claw,
      LightSensor cubeLightSensor,
      LightSensor coneLightSensor) {
    SmartDashboard.putBoolean("Is it coast", robotSubsystem.isCoast());
    SmartDashboard.putNumber("PICKUP TIMER", limelight.pickupTimer.get());
    SmartDashboard.putBoolean("PICKUPTIEMR CODE RAN THROUGH", false);
    if (limelight.pickupTimer.get() >= 0.9) {
      SmartDashboard.putBoolean("PICKUPTIEMR CODE RAN THROUGH", true);
      robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(0, 0, 0), true);
      claw.setHigh();

      limelight.pickupTimer.stop();
      limelight.pickupTimer.reset();
    }
    // limelight commands below

    if (armController.getPOV() == 0) {
      limelight.turnToAngle(robotSubsystem);
    }

    if (armController.getRawButtonPressed(Constants.Gamepad.L_JOYSTICK_BUTTON)
        || armController.getRawButtonPressed(Constants.Gamepad.R_JOYSTICK_BUTTON)) {
      claw.setLow();
      limelight.turnController.setP(0.0025);
      limelight.isTurningDone = false;
      limelight.pickupTimer.stop(); // just in case
      limelight.pickupTimer.reset();
      limelight.scoringTimer.stop(); // just in case
      limelight.scoringTimer.reset();
      robotSubsystem.setBrakeMode();
    }
    // if(armController.getRawButtonPressed(Constants.left_Button)){
    //     limelight.isTurningDone = false;
    //     limelight.turnController.setP(0.001);
    // }
    if (armController.getRawButtonReleased(Constants.Gamepad.L_JOYSTICK_BUTTON)
        || armController.getRawButtonReleased(Constants.Gamepad.R_JOYSTICK_BUTTON)) {
      robotSubsystem.setCoastMode();
    }

    if (armController.getRawButton(Constants.Gamepad.L_JOYSTICK_BUTTON) && !claw.isClawClosed) {
      limelight.pickup(robotSubsystem, arm, claw, cubeLightSensor, coneLightSensor, false, false);
    } else if (armController.getRawButton(Constants.Gamepad.R_JOYSTICK_BUTTON)
        && !claw.isClawClosed) {
      limelight.pickup(robotSubsystem, arm, claw, cubeLightSensor, coneLightSensor, true, true);
    }
    // else if (armController.getRawButton(Constants.left_Button)) {
    //     limelight.score(robotSubsystem, arm, claw, true);
    // }

    else { // no limelight commands`
      if (driveController.getRawButtonPressed(Constants.Gamepad.X_BUTTON)) {
        robotSubsystem.resetGyro();
      }

      if (armController.getRawButton(Constants.Gamepad.LEFT_BUTTON)) {
        if (robotSubsystem.isCoast()) {
          robotSubsystem.setBrakeMode();
        } else {
          robotSubsystem.setCoastMode();
        }
      }

      robotSubsystem.drive(
          new SwerveSubsystem.SwerveRequest(
              driveController.getRawAxis(Constants.Gamepad.RIGHT_JOYSTICK_XAXIS),
              -driveController.getRawAxis(Constants.Gamepad.LEFT_JOYSTICK_XAXIS),
              -driveController.getRawAxis(Constants.Gamepad.LEFT_JOYSTICK_YAXIS)),
          true);
    }
  }

  public void arm(ArmSubsystem arm) {
    SmartDashboard.putNumber("Encoder value big arm", arm.bigArmMotor.getPosition().getValue());
    SmartDashboard.putNumber("Encoder value small arm", arm.smallArmMotor.getPosition().getValue());
    if (highTargetTimer.get() >= 0.25) {
      arm.highTarget2();
      highTargetTimer.stop();
      highTargetTimer.reset();
    }
    if (sliderTimer.get() >= 0.7) { // redo the timing for this
      arm.sliderTarget2();
      sliderTimer.stop();
      sliderTimer.reset();
    }
    if (defaultTimer.get() >= 0.3) {
      arm.defaultTarget2();
      defaultTimer.stop();
      defaultTimer.reset();
    }
    if (pickupFallenConeTimer.get() >= 0.4) {
      arm.pickupFallenCone2();
      pickupFallenConeTimer.stop();
      pickupFallenConeTimer.reset();
    }

    if (driveController.getRawButtonPressed(Constants.Gamepad.R_JOYSTICK_BUTTON)
        || armController.getRawButtonPressed(Constants.Gamepad.X_BUTTON)) {
      arm.defaultTarget1();
      defaultTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.Gamepad.L_JOYSTICK_BUTTON)
        || armController.getRawButtonPressed(Constants.Gamepad.A_BUTTON)) {
      arm.pickupTarget();
    } else if (driveController.getRawButtonPressed(Constants.Gamepad.START_BUTTON)
        || armController.getRawButtonPressed(Constants.Gamepad.Y_BUTTON)) {
      arm.highTarget1();
      highTargetTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.Gamepad.Y_BUTTON)
        || armController.getRawButtonPressed(Constants.Gamepad.START_BUTTON)) {
      arm.sliderTarget1();
      sliderTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.Gamepad.BACK_BUTTON)
        || armController.getRawButtonPressed(Constants.Gamepad.B_BUTTON)) {
      arm.mediumTarget();
      // } else if (armController.getRawButtonPressed(Constants.Back_Button)) {

      //   arm.pickupFallenCone1();
      //   pickupFallenConeTimer.start();
    } else if (driveController.getRawAxis(Constants.Gamepad.LEFT_TRIGGER) > 0.05) {
      arm.setBigArmSpeed(-Constants.Speed.ARM_SPEED);
    } else if (driveController.getRawButton(Constants.Gamepad.LEFT_BUTTON)) {
      arm.setBigArmSpeed(Constants.Speed.ARM_SPEED);
    } else if (driveController.getRawAxis(Constants.Gamepad.RIGHT_TRIGGER) > 0.05) {
      arm.setSmallArmSpeed(-Constants.Speed.ARM_SPEED - .001);
    } else if (driveController.getRawButton(Constants.Gamepad.RIGHT_BUTTON)) {
      arm.setSmallArmSpeed(Constants.Speed.ARM_SPEED + .001);
    } else {
      arm.maintainPosition();
    }
  }
}
