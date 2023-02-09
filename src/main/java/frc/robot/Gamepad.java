package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.ArmSubsystem;
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

    public void armPneumatics(ClawPneumatics clawPneumatics) {
        SmartDashboard.putBoolean("if True then not full yet", clawPneumatics.isNotFull());

        if(driveController.getRawButton(Constants.Y_Button)) {
            clawPneumatics.setHigh();
        }
        else if(driveController.getRawButton(Constants.A_Button)) {
            clawPneumatics.setLow();
        } 
        else {
            clawPneumatics.setOff();
        }
        

        if (driveController.getRawButton(Constants.X_Button)) {
            clawPneumatics.enableCompressor();
        }

        if (driveController.getRawButton(Constants.B_Button)) {
            clawPneumatics.disableCompressor();
        }
    }

    public void swerveDrive(SwerveSubsystem robotSubsystem) {
        robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(
        driveController.getRawAxis(Constants.rightJoystickXAxis), 
        -driveController.getRawAxis(Constants.leftJoystickXAxis), 
        -driveController.getRawAxis(Constants.leftJoystickYAxis)));
    }

    public void arm(ArmSubsystem arm) {
        if(driveController.getPOV() == 0){
            arm.highTarget();
        }
        else if(driveController.getPOV() == 90){
            arm.mediumTarget();
        }
        else if(driveController.getPOV() == 180){
            arm.pickupTarget();
        }


        else if (driveController.getRawAxis(Constants.leftTrigger) > 0.05) {
            arm.setBigArmSpeed(Constants.ARM_SPEED);
        }
        else if(driveController.getRawButton(Constants.left_Button)) {
            arm.setBigArmSpeed(-Constants.ARM_SPEED);
        }


        else if (driveController.getRawAxis(Constants.rightTrigger) > 0.05) {
            arm.setSmallArmSpeed(Constants.ARM_SPEED);
        }
        else if(driveController.getRawButton(Constants.right_Button)) {
            arm.setSmallArmSpeed(-Constants.ARM_SPEED);
        }


        else{
            arm.stop();
        }
    }
}