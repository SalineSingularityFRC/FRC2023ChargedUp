package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.SwerveSubsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * 
 * Main class to control the robot
 * 
 */
public class Gamepad {

    String allianceColor = DriverStation.getAlliance().toString();
    
    private Joystick driveController;
    private Joystick armController;

    /**
     * 
     * @param driveControllerPort Controller port the drive controller is connected to, probably 0
     * @param armControllerPort Controller port the arm controller is connect to, probably 1
     */
    public Gamepad(int driveControllerPort, int armControllerPort) {
        driveController = new Joystick(driveControllerPort);
        armController = new Joystick(armControllerPort);
    }

    // public void armPneumatics(ArmPneumatics armPneumatics) {
    //     // SmartDashboard.putNumber("Pressure Value", 3);

    //     if (armController.getLB()) {
    //         armPneumatics.setLeftLow();
    //     }
    //     else if (armController.getTriggerLeft() > .2) {
    //         armPneumatics.setLeftHigh();
    //     }
    //     else {
    //         armPneumatics.setLeftOff();
    //     }

    //     if (armController.getRB()) {
    //         armPneumatics.setRightLow();
    //     }
    //     else if (armController.getTriggerRight() > .2) {
    //         armPneumatics.setRightHigh();
    //     }
    //     else {
    //         armPneumatics.setRightOff();
    //     }

    //     if (armController.getPOVUp()) {
    //         armPneumatics.enableCompressor();
    //     }

    //     if (armController.getPOVDown() ) { // double check this later
    //         armPneumatics.disableCompressor();
    //     }
    // }

    public void swerveDrive(SwerveSubsystem robotSubsystem) {
        robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(
        driveController.getRawAxis(Constants.rightJoystickXAxis), 
        -driveController.getRawAxis(Constants.leftJoystickXAxis), 
        -driveController.getRawAxis(Constants.leftJoystickYAxis)));
    }

    // public void arm() {
    //     if(joystick.getPOV()==0){
    //         bigArm.highTarget();
    //         smallArm.highTarget();
    //       }
    //       else if(joystick.getPOV() == 90){
    //         bigArm.mediumTarget();
    //         smallArm.mediumTarget();
      
    //       }
    //       else if(joystick.getPOV() == 180){
    //         bigArm.pickupTarget();
    //         smallArm.mediumTarget();
      
    //       }
    //       else{
    //         bigArm.stop();
    //         smallArm.stop();
    //       }
      
    // }

}