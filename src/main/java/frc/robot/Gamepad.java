package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * Main class to control the robot
 * 
 */
public class Gamepad {

    public String allianceColor = DriverStation.getAlliance().toString();

    private Timer highTargetTimer = new Timer();
    private Timer sliderTimer = new Timer();
    
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

    public void swerveDrive(SwerveSubsystem robotSubsystem, Limelight limelight, ArmSubsystem arm, ClawPneumatics claw) {
        SmartDashboard.putBoolean("Is it coast", robotSubsystem.isCoast());
        // limelight commands below
        if (armController.getRawButtonPressed(Constants.A_Button) || armController.getRawButtonPressed(Constants.B_Button)) {
            limelight.isTurningDone = false;
            robotSubsystem.setBrakeMode();
        }
        if (armController.getRawButtonReleased(Constants.A_Button) || armController.getRawButtonReleased(Constants.B_Button)) {
            robotSubsystem.setCoastMode();
        }

        if (armController.getRawButton(Constants.A_Button)) {
            limelight.pickup(robotSubsystem, arm, claw, true);
        }  
        else if (armController.getRawButton(Constants.B_Button)) {
            limelight.pickup(robotSubsystem, arm, claw, false);
        }

        else { // no limelight commands
            if (driveController.getRawButtonPressed(Constants.X_Button)) {
                robotSubsystem.resetGyro();
            }
            if (armController.getRawButtonPressed(Constants.Start_Button)) {
                if (robotSubsystem.isCoast()) {
                    robotSubsystem.setBrakeMode();
                }
                else {
                    robotSubsystem.setCoastMode();
                }
            }
    
            robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(
            driveController.getRawAxis(Constants.rightJoystickXAxis), 
            -driveController.getRawAxis(Constants.leftJoystickXAxis), 
            -driveController.getRawAxis(Constants.leftJoystickYAxis)), true);
        }
    }

    public void arm(ArmSubsystem arm) {
        SmartDashboard.putNumber("Encoder value big arm", arm.bigArmMotor.getPosition().getValue());
        SmartDashboard.putNumber("Encoder value small arm", arm.smallArmMotor.getPosition().getValue());
        if (highTargetTimer.get() >= 0.7) {
            arm.highTarget2();
            highTargetTimer.stop();
            highTargetTimer.reset();
        }

        if (sliderTimer.get() >= 0.7) {
            arm.sliderTarget2();
            sliderTimer.stop();
            sliderTimer.reset();
        } // redo the timing for this

        if (driveController.getRawButtonPressed(Constants.R_joystick_Button)) {
            arm.defaultTarget();
        }
        else if(driveController.getRawButtonPressed(Constants.L_joystick_Button)) {
            arm.pickupTarget();
        }
        else if (driveController.getRawButtonPressed(Constants.Start_Button)) {
            arm.highTarget1();
            highTargetTimer.start();
        }
        else if(driveController.getRawButtonPressed(Constants.Y_Button)){
            arm.sliderTarget1();
            sliderTimer.start();
        }
        else if(driveController.getRawButtonPressed(Constants.Back_Button)) {
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