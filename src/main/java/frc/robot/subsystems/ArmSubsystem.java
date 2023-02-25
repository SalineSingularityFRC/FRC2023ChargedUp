package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.PositionVoltage;
import frc.robot.Constants;

import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.Slot1Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenixpro.controls.VelocityVoltage;

public class ArmSubsystem {
    public TalonFX smallArmMotor;
    public TalonFX bigArmMotor;
    public TalonFX bigArmMotor2;

    private MotionMagicVoltage positionTargetPreset = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private MotionMagicVoltage positionTargetManual = new MotionMagicVoltage(0).withSlot(1).withEnableFOC(true);

    private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
    private TalonFXConfiguration talonFXConfigsManual = new TalonFXConfiguration();

    private MotionMagicConfigs motionMagicConfigsPresets;
    private MotionMagicConfigs motionMagicConfigsManual;


    private double bigArmPos;
    private double smallArmPos;

    private final double presetSmallP = 4.0;
    private final double presetSmallI = 0.08;
    private final double presetSmallD = 0.16;
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
        motionMagicConfigsPresets.MotionMagicAcceleration = 200;
        motionMagicConfigsPresets.MotionMagicJerk = 1800;

        // motionMagicConfigsManual = talonFXConfigsManual.MotionMagic;
        // motionMagicConfigsManual.MotionMagicCruiseVelocity = 6;
        // motionMagicConfigsManual.MotionMagicAcceleration = 40;
        // motionMagicConfigsManual.MotionMagicJerk = 800;
        
        bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);
        smallArmMotor.getConfigurator().apply(motionMagicConfigsPresets);

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void setSmallArmSpeed(double speed){
        smallArmPos = smallArmMotorPosition + speed;
        smallArmMotor.setControl(positionTargetManual.withPosition(smallArmPos));
        // smallArmMotor.setControl(m_voltageVelocity.withVelocity(speed));
        
        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void setBigArmSpeed(double speed) {
        bigArmPos = bigArmMotorPosition + speed;
        bigArmMotor.setControl(positionTargetManual.withPosition(bigArmPos));
        // bigArmMotor.setControl(m_voltageVelocity.withVelocity(speed));

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }




    public void setPosition(double smallArmAngle, double bigArmAngle) {
       smallArmPosition(smallArmAngle);
       bigArmPosition(bigArmAngle);
    }

    public void smallArmPosition(double smallArmAngle) {
        smallArmMotor.setControl(positionTargetPreset.withPosition(smallArmAngle).withFeedForward(0.05));

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void bigArmPosition(double bigArmAngle) {
        bigArmMotor.setControl(positionTargetPreset.withPosition(bigArmAngle).withFeedForward(0.05));

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }



    public void highTarget(){
        setPosition(Constants.SmallArm_highTarget , Constants.BigArm_highTarget);
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
