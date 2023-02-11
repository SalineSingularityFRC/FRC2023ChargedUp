package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.controls.PositionVoltage;
import frc.robot.Constants;
import com.ctre.phoenixpro.configs.Slot0Configs;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenixpro.controls.VelocityVoltage;

public class ArmSubsystem {
    public TalonFX smallArmMotor;
    public TalonFX bigArmMotor;
    private PositionVoltage positionTarget;

    private final double kP = 2.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double bigArm_kS = 0.028;//counters gravity

    private double bigArmMotorPosition;
    private double smallArmMotorPosition;

    /* Be able to switch which control request to use based on a button press */
    /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, true, 0, 0, false);
    
    public ArmSubsystem(boolean bigArmIsInverted, boolean smallArmIsInverted) {
        smallArmMotor = new TalonFX(Constants.SMALL_ARM_MOTOR_ID, Constants.CANBUS);
        smallArmMotor.setInverted(smallArmIsInverted);

        bigArmMotor = new TalonFX(Constants.BIG_ARM_Motor_ID, Constants.CANBUS);
        bigArmMotor.setInverted(bigArmIsInverted);
        
        positionTarget = new PositionVoltage(0).withSlot(0);
        
        //Big Arm Slot Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP; 
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        bigArmMotor.getConfigurator().apply(slot0Configs);

        //Small Arm Slot Configuration
        slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP; 
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        smallArmMotor.getConfigurator().apply(slot0Configs);

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void stop() {
        smallArmMotor.set(0); 
        bigArmMotor.set(0); 
    }

    public void setSpeed(double smallArmSpeed, double bigArmSpeed) { //speed will be from -1.0 to 1.0
        smallArmMotor.setControl(m_voltageVelocity.withVelocity(smallArmSpeed));
        bigArmMotor.setControl(m_voltageVelocity.withVelocity(bigArmSpeed));

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void setSmallArmSpeed(double speed){
        smallArmMotor.setControl(m_voltageVelocity.withVelocity(speed));

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void setBigArmSpeed(double speed){
        bigArmMotor.setControl(m_voltageVelocity.withVelocity(speed));

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }




    public void setPosition(double smallArmAngle, double bigArmAngle) {
        smallArmPosition(smallArmAngle);
        bigArmPosition(bigArmAngle);
    }

    public void smallArmPosition(double smallArmAngle) {
        smallArmMotor.setControl(positionTarget.withPosition(smallArmAngle));

        bigArmMotorPosition = bigArmMotor.getPosition().getValue();
        smallArmMotorPosition = smallArmMotor.getPosition().getValue();
    }

    public void bigArmPosition(double bigArmAngle) {
        bigArmMotor.setControl(positionTarget.withPosition(bigArmAngle));

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
        setPosition(0, 0);

    }


    public void maintainPosition() {
        smallArmMotor.setControl(positionTarget.withPosition(smallArmMotorPosition));
        bigArmMotor.setControl(positionTarget.withPosition(bigArmMotorPosition));
    }

}
