package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.Slot1Configs;
import com.ctre.phoenixpro.configs.Slot2Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;

public class ArmSubsystem {
  public TalonFX smallArmMotor;
  public TalonFX bigArmMotor;
  public TalonFX bigArmMotor2;

  private MotionMagicVoltage positionTargetPreset =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

  private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
  private TalonFXConfiguration talonFXConfigsManual = new TalonFXConfiguration();

  private MotionMagicConfigs motionMagicConfigsPresets;
  private MotionMagicConfigs motionMagicConfigsPresetsSmall;

  private MotionMagicConfigs motionMagicConfigsManual;

  private double bigArmPos;
  private double smallArmPos;

  private final double presetSmallP = 2.0 * 20;
  private final double presetSmallI = 0.02 * 10;
  private final double presetSmallD = 0.02 * 10;
  private final double presetSmallS = 0.06 * 10;

  private final double presetBigP = 8.0 * 14;
  private final double presetBigI = 0.08 * 10;
  private final double presetBigD = 0.08 * 10;
  private final double presetBigS = 0.06 * 10;

  private final double manualP = 8 * 24;
  private final double manualI = 0.8 * 14;
  private final double manualD = 0.8;
  private final double manualS = 0.6; // counters static friction

  public double bigArmMotorPosition;
  public double smallArmMotorPosition;

  public ArmSubsystem(boolean bigArmIsInverted, boolean smallArmIsInverted) {
    smallArmMotor = new TalonFX(Constants.SMALL_ARM_MOTOR_ID, Constants.CANBUS);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = Constants.SMALL_ARM_CANCODER_ID;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    smallArmMotor.getConfigurator().apply(config);
    smallArmMotor.setInverted(smallArmIsInverted);

    bigArmMotor = new TalonFX(Constants.BIG_ARM_Motor_ID, Constants.CANBUS);

    config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = Constants.BIG_ARM_CANCODER_ID;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    bigArmMotor.getConfigurator().apply(config);
    bigArmMotor.setInverted(bigArmIsInverted);

    bigArmMotor2 = new TalonFX(Constants.BIG_ARM_Motor_2_ID, Constants.CANBUS);
    bigArmMotor2.setControl(new Follower(Constants.BIG_ARM_Motor_ID, true));

    Slot0Configs slot0ConfigsSmall = new Slot0Configs();
    Slot0Configs slot0ConfigsBig = new Slot0Configs();
    slot0ConfigsSmall.kP = presetSmallP;
    slot0ConfigsSmall.kI = presetSmallI;
    slot0ConfigsSmall.kD = presetSmallD;
    slot0ConfigsSmall.kS = presetSmallS;
    slot0ConfigsBig.kP = presetBigP;
    slot0ConfigsBig.kI = presetBigI;
    slot0ConfigsBig.kD = presetBigD;
    slot0ConfigsBig.kS = presetBigS;

    Slot1Configs slot1Configs = new Slot1Configs();
    slot1Configs.kP = manualP;
    slot1Configs.kI = manualI;
    slot1Configs.kD = manualD;
    slot1Configs.kS = manualS;

    Slot2Configs slot2Configs = new Slot2Configs();
    slot1Configs.kP = manualP;
    slot1Configs.kI = manualI;
    slot1Configs.kD = manualD;
    slot1Configs.kS = manualS;

    bigArmMotor.getConfigurator().apply(slot0ConfigsBig);
    smallArmMotor.getConfigurator().apply(slot0ConfigsSmall);

    bigArmMotor.getConfigurator().apply(slot1Configs);
    smallArmMotor.getConfigurator().apply(slot1Configs);

    bigArmMotor.getConfigurator().apply(slot2Configs);
    smallArmMotor.getConfigurator().apply(slot2Configs);

    motionMagicConfigsPresets = talonFXConfigsPreset.MotionMagic;
    motionMagicConfigsPresets.MotionMagicCruiseVelocity = 40 / 30;
    motionMagicConfigsPresets.MotionMagicAcceleration = 100 / 40;
    motionMagicConfigsPresets.MotionMagicJerk = 900 / 30;

    motionMagicConfigsManual = talonFXConfigsManual.MotionMagic;
    motionMagicConfigsManual.MotionMagicCruiseVelocity = 40 / 30;
    motionMagicConfigsManual.MotionMagicAcceleration = 70 / 40;
    motionMagicConfigsManual.MotionMagicJerk = 800 / 30;

    bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);

    motionMagicConfigsPresetsSmall = talonFXConfigsPreset.MotionMagic;
    motionMagicConfigsPresetsSmall.MotionMagicJerk = 900 / 30;
    motionMagicConfigsPresetsSmall.MotionMagicAcceleration = 100 / 50;
    motionMagicConfigsPresetsSmall.MotionMagicCruiseVelocity = .7;

    smallArmMotor.getConfigurator().apply(motionMagicConfigsPresetsSmall);

    bigArmMotorPosition = bigArmMotor.getPosition().getValue();
    smallArmMotorPosition = smallArmMotor.getPosition().getValue();
  }

  public void setSmallArmSpeed(double speed) {
    // smallArmMotor.getConfigurator().apply(motionMagicConfigsManual);
    smallArmPos = smallArmMotorPosition + speed;
    if (speed < 0) {
      smallArmMotor.setControl(
          positionTargetPreset.withPosition(smallArmPos).withFeedForward(0.05).withSlot(2));

    } else {
      smallArmMotor.setControl(
          positionTargetPreset.withPosition(smallArmPos).withFeedForward(0.05).withSlot(1));
    }

    bigArmMotorPosition = bigArmMotor.getPosition().getValue();
    smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    SmartDashboard.putNumber("SMALL ARM POSITION MANUAL", smallArmMotorPosition);
  }

  public void setBigArmSpeed(double speed) {
    // bigArmMotor.getConfigurator().apply(motionMagicConfigsManual);
    bigArmPos = bigArmMotorPosition + speed;
    if (speed < 0) {
      bigArmMotor.setControl(
          positionTargetPreset.withPosition(bigArmPos).withFeedForward(0.05).withSlot(2));

    } else {
      bigArmMotor.setControl(
          positionTargetPreset.withPosition(bigArmPos).withFeedForward(0.05).withSlot(1));
    }

    bigArmMotorPosition = bigArmMotor.getPosition().getValue();
    smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    SmartDashboard.putNumber("BIG ARM POSITION MANUAL", bigArmMotorPosition);
  }

  public void setPosition(double smallArmAngle, double bigArmAngle) {
    smallArmPosition(smallArmAngle);
    bigArmPosition(bigArmAngle);
  }

  public void smallArmPosition(double smallArmAngle) {
    smallArmMotor.getConfigurator().apply(motionMagicConfigsPresetsSmall);
    smallArmMotor.setControl(
        positionTargetPreset.withPosition(smallArmAngle).withFeedForward(0.05).withSlot(0));

    smallArmMotorPosition = smallArmAngle;
  }

  public void bigArmPosition(double bigArmAngle) {
    bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);
    bigArmMotor.setControl(
        positionTargetPreset.withPosition(bigArmAngle).withFeedForward(0.05).withSlot(0));

    bigArmMotorPosition = bigArmAngle;
  }

  public void highTarget(
      Timer timer) { // these will still be used for auton and limelight, which have the luxury of
    // calling this method over and over
    highTarget1();
    if (timer.get() >= 0.25) {
      highTarget2();
    }
  }

  public void
      highTarget1() { // these individual commands labeled 1 and 2 are for gamepad to call (so you
    // only need to press it once)
    bigArmPosition(Constants.BigArm_highTarget);
  }

  public void highTarget2() {
    smallArmPosition(Constants.SmallArm_highTarget);
  }

  public void sliderTarget(Timer timer) { // same comment as highTarget
    sliderTarget1();
    if (timer.get() >= 0.7) {
      sliderTarget2();
    }
  }

  public void sliderTarget1() { // same comment as highTarget1
    bigArmPosition(Constants.BigArm_slider);
  }

  public void sliderTarget2() {
    smallArmPosition(Constants.SmallArm_Slider);
  }

  public void mediumTarget() {
    setPosition(Constants.SmallArm_mediumTarget, Constants.BigArm_mediumTarget);
  }

  public void pickupTarget() {
    setPosition(Constants.SmallArm_pickup, Constants.BigArm_pickup);
  }

  public void pickupFallenCone1() {
    bigArmPosition(Constants.BigArm_pickupCone);
  }

  public void pickupFallenCone2() {
    smallArmPosition(Constants.SmallArm_pickupCone);
  }

  public void defaultTarget() {
    setPosition(Constants.SmallArm_default, Constants.BigArm_default);
  }

  public void defaultTargetTimer(Timer timer) {
    defaultTarget1();
    if (timer.get() >= 0.7) {
      defaultTarget2();
    }
  }

  public void defaultTarget1() {
    smallArmPosition(Constants.SmallArm_default);
  }

  public void defaultTarget2() {
    bigArmPosition(Constants.BigArm_default);
  }

  public void maintainPosition() {
    bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);
    smallArmMotor.getConfigurator().apply(motionMagicConfigsPresetsSmall);

    smallArmMotor.setControl(positionTargetPreset.withPosition(smallArmMotorPosition).withSlot(0));
    bigArmMotor.setControl(positionTargetPreset.withPosition(bigArmMotorPosition).withSlot(0));

    SmartDashboard.putNumber("big arm pos", bigArmMotorPosition);
    SmartDashboard.putNumber("small arm pos", smallArmMotorPosition);
  }
}
