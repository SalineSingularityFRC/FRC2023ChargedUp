package frc.robot.Arm;

import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.controls.PositionVoltage;
import frc.robot.Constants;

public class Arm {

    private TalonFX driveMotor;
    private PositionVoltage positionTarget;
    
    public Arm(int Can_ID_driveMotor, String canNetwork, boolean isInverted) {
        driveMotor = new TalonFX(Can_ID_driveMotor, canNetwork);
        driveMotor.setInverted(isInverted);
        positionTarget = new PositionVoltage(0).withSlot(0);
    }

    public void setSpeed(double speed) { //speed will be from -1.0 to 1.0
        driveMotor.set(speed);
    }

    public void setPosition(double angle) {
        driveMotor.setControl(positionTarget.withPosition(Constants.ANGLE_MOTOR_GEAR_RATIO * (angle/(2*Math.PI))));
    }

    public void coast() {
        driveMotor.set(0); // this is for when the joystick is not being moved at all
    }
}
