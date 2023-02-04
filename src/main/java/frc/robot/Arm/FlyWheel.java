package frc.robot.Arm;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;

public class FlyWheel {
    Falcon climbMotor;

    private CANcoder m_encoder;
    private TalonFX driveMotor;

    double kP = 0.02;
    double kI = 0.0000;
    double kD = 0.0;
    double kIz = 0;
    double kFF = 0.0;
    double kMaxOutput = .5;
    double kMinOutput = -.5;
    double maxRPMIntake = 11000;
    // maxRPMIntakeconveyor is copied from 2021 conveyor class
    double maxRPMIntakeconveyor = -4000;
    double maxRPMFeed = 5700;
    double deployPosition = 312761.000000;
    double retractPosition = 8350; //~115 degrees to the 


    public Climber (int climberDeployPort) {
        climbMotor = new Falcon(climberDeployPort, 1, false, false, "rio", kP, kI, kD, kFF, kMaxOutput, kMinOutput);

    }

    public void deploy () {
        climbMotor.setPosition(deployPosition);
    }

    public void climb () {
        climbMotor.setSpeed(-.70);

    }

    public void stop () {
        climbMotor.setSpeed(0.0);
    }

    public void reset () {
        climbMotor.setSpeed(.70);
    }

    public double getPosition(){
        return climbMotor.getSelectedSensorPosition();
    }
}
