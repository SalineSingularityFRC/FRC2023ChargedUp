package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.ControlModeValue;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;

import frc.robot.Constants;

import java.time.Duration;

import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.Slot1Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenixpro.controls.VelocityVoltage;

public class ArmSubsystem {
    public TalonFX smallArmMotor;
    public TalonFX bigArmMotor;
    public TalonFX bigArmMotor2;

    public CANcoder bigArmEncoder;

    private MotionMagicVoltage positionTargetPreset = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    // private MotionMagicVoltage positionTargetManual = new MotionMagicVoltage(0).withSlot(1).withEnableFOC(true);

    // private TorqueCurrentFOC torqueDrive = new TorqueCurrentFOC(0).withDeadband(1).withMaxAbsDutyCycle(.4);

    private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
    private TalonFXConfiguration talonFXConfigsManual = new TalonFXConfiguration();

    private MotionMagicConfigs motionMagicConfigsPresets;
    private MotionMagicConfigs motionMagicConfigsManual;


    private double bigArmPos;
    private double smallArmPos;

    private final double presetSmallP = 2.0;
    private final double presetSmallI = 0.02;
    private final double presetSmallD = 0.02;
    private final double presetSmallS = 0.06;

    private final double presetBigP = 8.0;
    private final double presetBigI = 0.08;
    private final double presetBigD = 0.08;
    private final double presetBigS = 0.06;

    private final double manualP = 4.0;
    private final double manualI = 0.0;
    private final double manualD = 0.0;
    private final double manualS = 0; // counters static friction

    private double bigArmMotorPosition;
    private double smallArmMotorPosition;
    
    public ArmSubsystem(boolean bigArmIsInverted, boolean smallArmIsInverted) {
        smallArmMotor = new TalonFX(Constants.SMALL_ARM_MOTOR_ID, Constants.CANBUS);
        smallArmMotor.setInverted(smallArmIsInverted);

        bigArmMotor = new TalonFX(Constants.BIG_ARM_Motor_ID, Constants.CANBUS);
        bigArmMotor.setInverted(bigArmIsInverted);

        bigArmMotor2 = new TalonFX(Constants.BIG_ARM_Motor_2_ID, Constants.CANBUS);
        bigArmMotor2.setControl(new Follower(Constants.BIG_ARM_Motor_ID, true));

        // bigArmEncoder = new CANcoder(, );


        
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


        bigArmMotor.getConfigurator().apply(slot0ConfigsBig);
        smallArmMotor.getConfigurator().apply(slot0ConfigsSmall);

        bigArmMotor.getConfigurator().apply(slot1Configs);
        smallArmMotor.getConfigurator().apply(slot1Configs);


        motionMagicConfigsPresets = talonFXConfigsPreset.MotionMagic;
        motionMagicConfigsPresets.MotionMagicCruiseVelocity = 40;
        motionMagicConfigsPresets.MotionMagicAcceleration = 100;
        motionMagicConfigsPresets.MotionMagicJerk = 900;

        motionMagicConfigsManual = talonFXConfigsManual.MotionMagic;
        motionMagicConfigsManual.MotionMagicCruiseVelocity = 6;
        motionMagicConfigsManual.MotionMagicAcceleration = 40;
        motionMagicConfigsManual.MotionMagicJerk = 800;
        
        bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);
        smallArmMotor.getConfigurator().apply(motionMagicConfigsPresets);

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void setSmallArmSpeed(double speed){
        smallArmMotor.getConfigurator().apply(motionMagicConfigsManual);

        smallArmPos = smallArmMotorPosition + speed;
        smallArmMotor.setControl(positionTargetPreset.withPosition(smallArmPos).withFeedForward(0.05)); // feed forward counters gravity
        
        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void setBigArmSpeed(double speed) {
        bigArmMotor.getConfigurator().apply(motionMagicConfigsManual);

        bigArmPos = smallArmMotorPosition + speed;
        bigArmMotor.setControl(positionTargetPreset.withPosition(bigArmPos).withFeedForward(0.05));         

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }




    public void setPosition(double smallArmAngle, double bigArmAngle) {
       smallArmPosition(smallArmAngle);
       bigArmPosition(bigArmAngle);
    }

    public void smallArmPosition(double smallArmAngle) {
        smallArmMotor.getConfigurator().apply(motionMagicConfigsPresets);
        smallArmMotor.setControl(positionTargetPreset.withPosition(smallArmAngle).withFeedForward(0.05));

        smallArmMotorPosition = smallArmAngle;
    }

    public void bigArmPosition(double bigArmAngle) {
        bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);
        bigArmMotor.setControl(positionTargetPreset.withPosition(bigArmAngle).withFeedForward(0.05));

        bigArmMotorPosition = bigArmAngle;
    }



    public void highTarget(){
        setPosition(Constants.SmallArm_highTarget , Constants.BigArm_highTarget);
    } 
    
    public void autonHighTarget(Timer timer) {
        autonHighTarget1();
        if (timer.get() >= 0.7) {
            autonHighTarget2();        
        }
    }
    public void autonHighTarget1() {
        bigArmPosition(Constants.BigArm_highTarget);
    }
    public void autonHighTarget2() {
        smallArmPosition(Constants.SmallArm_highTarget);
    }

    public void sliderTarget1() {
        bigArmPosition(Constants.BigArm_slider);
    }
    public void sliderTarget2() {
        smallArmPosition(Constants.SmallArm_Slider);
    }

    public void mediumTarget(){
        setPosition(Constants.SmallArm_mediumTarget, Constants.BigArm_mediumTarget);
    }
    public void pickupTarget(){
        setPosition(Constants.SmallArm_pickup, Constants.BigArm_pickup);
    }
    public void defaultTarget(){
        setPosition(Constants.SmallArm_default, Constants.BigArm_default);
    }


    public void maintainPosition() {
        smallArmMotor.setControl(positionTargetPreset.withPosition(smallArmMotorPosition));
        bigArmMotor.setControl(positionTargetPreset.withPosition(bigArmMotorPosition));
    }

}
