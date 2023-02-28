package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

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

    // private boolean isConstantMode;

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

        if(driveController.getRawButtonPressed(Constants.A_Button)) {
            if (clawPneumatics.isClawClosed) {
                clawPneumatics.setLow();
            }
            else {
                clawPneumatics.setHigh();
            }
        }
        else if(driveController.getRawButtonReleased(Constants.A_Button)) {
            clawPneumatics.setOff();
        } 
        

        if (driveController.getRawButtonPressed(Constants.B_Button)) {
            clawPneumatics.toggleCompressor();
        }
    }

    public void swerveDrive(SwerveSubsystem robotSubsystem) {
        double divisor;

        if (driveController.getRawButton(Constants.Y_Button)) {
            divisor = Constants.SNAIL_SPEED;
        }
        else {
            divisor = 1;
        }

        if (driveController.getRawButtonPressed(Constants.X_Button)) {
            robotSubsystem.resetGyro();
        }

        robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(
        driveController.getRawAxis(Constants.rightJoystickXAxis) * divisor, 
        -driveController.getRawAxis(Constants.leftJoystickXAxis) * divisor, 
        -driveController.getRawAxis(Constants.leftJoystickYAxis) * divisor));
    }

    public void arm(ArmSubsystem arm) {
        SmartDashboard.putNumber("Encoder value big arm", arm.bigArmMotor.getPosition().getValue());
        SmartDashboard.putNumber("Encoder value small arm", arm.smallArmMotor.getPosition().getValue());
        // if(driveController.getPOV() == 0){
        //     arm.highTarget();
        // }
        // else if(driveController.getPOV() == 90){
        //     arm.mediumTarget();
        // }
        // else if(driveController.getPOV() == 180){
        //     arm.pickupTarget();
        // }
        // else if (driveController.getPOV() == 270) {
        //     arm.defaultTarget();
        // }

        if (driveController.getRawButton(Constants.R_joystick_Button)) {
            arm.defaultTarget();
        }
        else if(driveController.getRawButton(Constants.L_joystick_Button)) {
            arm.pickupTarget();
        }
        else if (driveController.getRawButton(Constants.Start_Button)) {
            arm.highTarget();
        }
        else if(driveController.getRawButton(Constants.Back_Button)) {
            arm.mediumTarget();
        }


        else if (driveController.getRawAxis(Constants.leftTrigger) > 0.05) {
            arm.setBigArmSpeed(-Constants.ARM_SPEED);
        }
        else if(driveController.getRawButton(Constants.left_Button)) {
            arm.setBigArmSpeed(Constants.ARM_SPEED);
        }


        else if (driveController.getRawAxis(Constants.rightTrigger) > 0.05) {
            arm.setSmallArmSpeed(Constants.ARM_SPEED);
        }
        else if(driveController.getRawButton(Constants.right_Button)) {
            arm.setSmallArmSpeed(-Constants.ARM_SPEED);
        }


        else{
            arm.maintainPosition();
        }
    }
}