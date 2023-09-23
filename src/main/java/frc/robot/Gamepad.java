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
      ClawPneumatics clawPneumatics, LightSensor coneLightSensor , LightSensor cubeLightSensor , ArmSubsystem arm) {
    // SmartDashboard.putBoolean("if True then not full yet", clawPneumatics.isNotFull());

    // if(armController.getRawButtonPressed(Constants.right_Button)) {
    //     candle.turnOn();
    // }
    // if(armController.getRawButtonReleased(Constants.right_Button)) {
    //     candle.turnOff();
    // }
    if (clawCloseTimer.get() >= 0.25) {
      if ((arm.smallArmMotorPosition + (Constants.ARM_SPEED * 4.5)) < Constants.SmallArm_default) {
        arm.smallArmMotorPosition += Constants.ARM_SPEED * 4.5;
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
    if (cubeLightSensor.isSensed() && (armController.getRawButton(Constants.right_Button))) {
      clawPneumatics.setHigh();
      clawCloseTimer.start();
     
    } else if (!coneLightSensor.isSensed() && (armController.getRawButton(Constants.Back_Button))) {
      clawPneumatics.setHigh();
      clawCloseTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.A_Button)) {
      if (clawPneumatics.isClawClosed) {
        if (arm.bigArmMotorPosition > Constants.BigArm_pickup) {
          arm.smallArmMotorPosition -= Constants.ARM_SPEED * 9;
        }
        arm.maintainPosition();
        clawOpenTimer.start();
      } else {
        clawPneumatics.setHigh();
        clawCloseTimer.start();
      }
    } else if (driveController.getRawButtonReleased(Constants.A_Button)) {
      clawPneumatics.setOff();
    }

    if (driveController.getRawButtonPressed(Constants.B_Button)) {
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

    if (armController.getRawButtonPressed(Constants.L_joystick_Button)
        || armController.getRawButtonPressed(Constants.R_joystick_Button)) {
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
    if (armController.getRawButtonReleased(Constants.L_joystick_Button)
        || armController.getRawButtonReleased(Constants.R_joystick_Button)) {
      robotSubsystem.setCoastMode();
    }

    if (armController.getRawButton(Constants.L_joystick_Button) && !claw.isClawClosed) {
      limelight.pickup(robotSubsystem, arm, claw, cubeLightSensor, coneLightSensor, false, false);
    } else if (armController.getRawButton(Constants.R_joystick_Button) && !claw.isClawClosed) {
      limelight.pickup(robotSubsystem, arm, claw, cubeLightSensor, coneLightSensor,true, true);
    }
    // else if (armController.getRawButton(Constants.left_Button)) {
    //     limelight.score(robotSubsystem, arm, claw, true);
    // }

    else { // no limelight commands`
      if (driveController.getRawButtonPressed(Constants.X_Button)) {
        robotSubsystem.resetGyro();
      }

      if (armController.getRawButton(Constants.left_Button)) {
        if (robotSubsystem.isCoast()) {
          robotSubsystem.setBrakeMode();
        } else {
          robotSubsystem.setCoastMode();
        }
      }

      robotSubsystem.drive(
          new SwerveSubsystem.SwerveRequest(
              driveController.getRawAxis(Constants.rightJoystickXAxis),
              -driveController.getRawAxis(Constants.leftJoystickXAxis),
              -driveController.getRawAxis(Constants.leftJoystickYAxis)),
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

    if (driveController.getRawButtonPressed(Constants.R_joystick_Button)
        || armController.getRawButtonPressed(Constants.X_Button)) {
      arm.defaultTarget1();
      defaultTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.L_joystick_Button)
        || armController.getRawButtonPressed(Constants.A_Button)) {
      arm.pickupTarget();
    } else if (driveController.getRawButtonPressed(Constants.Start_Button)
        || armController.getRawButtonPressed(Constants.Y_Button)) {
      arm.highTarget1();
      highTargetTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.Y_Button)
        || armController.getRawButtonPressed(Constants.Start_Button)) {
      arm.sliderTarget1();
      sliderTimer.start();
    } else if (driveController.getRawButtonPressed(Constants.Back_Button)
        || armController.getRawButtonPressed(Constants.B_Button)) {
      arm.mediumTarget();
    // } else if (armController.getRawButtonPressed(Constants.Back_Button)) {

    //   arm.pickupFallenCone1();
    //   pickupFallenConeTimer.start();
    } else if (driveController.getRawAxis(Constants.leftTrigger) > 0.05) {
      arm.setBigArmSpeed(-Constants.ARM_SPEED);
    } else if (driveController.getRawButton(Constants.left_Button)) {
      arm.setBigArmSpeed(Constants.ARM_SPEED);
    } else if (driveController.getRawAxis(Constants.rightTrigger) > 0.05) {
      arm.setSmallArmSpeed(-Constants.ARM_SPEED);
    } else if (driveController.getRawButton(Constants.right_Button)) {
      arm.setSmallArmSpeed(Constants.ARM_SPEED);
    } else {
      arm.maintainPosition();
    }
  }
}
