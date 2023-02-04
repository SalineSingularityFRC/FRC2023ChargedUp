package frc.robot.Arm;

import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.hardware.CANcoder;
import frc.robot.Constants;

public class Arm {

    private TalonFX driveMotor;
    
    public Arm(int Can_ID_driveMotor, int Can_ID_angleMotor, String canNetwork, boolean isInverted) {
        driveMotor = new TalonFX(Can_ID_driveMotor, canNetwork);
        driveMotor.setInverted(isInverted);
    }

    public void setSpeed(double speed) { //speed will be from -1.0 to 1.0
        driveMotor.set(speed);
    }

    public void setPosition(double angle) {
        
    }

    public void coast() {
        driveMotor.set(0); // this is for when the joystick is not being moved at all
    }
}
