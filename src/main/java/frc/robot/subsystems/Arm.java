package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.controls.PositionVoltage;
import frc.robot.Constants;
import com.ctre.phoenixpro.configs.Slot0Configs;

public class Arm {

    private TalonFX armMotor;
    private PositionVoltage positionTarget;

    private final double kP = 2.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    
    public Arm(int armMotor_CAN_ID, String canNetwork, boolean isInverted) {
        armMotor = new TalonFX(armMotor_CAN_ID, canNetwork);
        armMotor.setInverted(isInverted);

        positionTarget = new PositionVoltage(0).withSlot(0);
        
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP; 
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        armMotor.getConfigurator().apply(slot0Configs);
    }

    public void setSpeed(double speed) { //speed will be from -1.0 to 1.0
        armMotor.set(speed);
    }

    public void setPosition(double angle) {
        armMotor.setControl(positionTarget.withPosition(Constants.ANGLE_MOTOR_GEAR_RATIO * (angle/(2*Math.PI))));
    }

    public void stop() {
        armMotor.set(0); 
    }
}
