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
    private Timer defaultTimer = new Timer();
    
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

    public void armPneumatics(ClawPneumatics clawPneumatics, LightSensor lightSensor, CANdleSystem candle) {
        SmartDashboard.putBoolean("if True then not full yet", clawPneumatics.isNotFull());

        if(armController.getRawButtonPressed(Constants.right_Button)) {
            candle.turnOn();
        }
        if(armController.getRawButtonReleased(Constants.right_Button)) {
            candle.turnOff();
        }
        if(lightSensor.isSensed() && (armController.getRawButton(Constants.right_Button) || armController.getRawButton(Constants.left_Button))) {
            clawPneumatics.setHigh();
        }
        
        else if(driveController.getRawButtonPressed(Constants.A_Button)) {
            if (clawPneumatics.isClawClosed) {
                clawPneumatics.setHigh();
            }
            else {
                clawPneumatics.setLow();
            }
        }
        else if(driveController.getRawButtonReleased(Constants.A_Button)) {
            clawPneumatics.setOff();
        } 
        

        if (driveController.getRawButtonPressed(Constants.B_Button)) {
            clawPneumatics.toggleCompressor();
        }
    }

    public void swerveDrive(SwerveSubsystem robotSubsystem, Limelight limelight, ArmSubsystem arm, ClawPneumatics claw, LightSensor lightSensor) {
        SmartDashboard.putBoolean("Is it coast", robotSubsystem.isCoast());
        // limelight commands below
        if (armController.getRawButtonPressed(Constants.A_Button) || armController.getRawButtonPressed(Constants.B_Button)
                || armController.getRawButtonPressed(Constants.X_Button)) {
            limelight.isTurningDone = false;
            limelight.scoringTimer.stop(); // just in case
            limelight.scoringTimer.reset();
            robotSubsystem.setBrakeMode();
        }
        if (armController.getRawButtonReleased(Constants.A_Button) || armController.getRawButtonReleased(Constants.B_Button)
                || armController.getRawButtonReleased(Constants.X_Button)) {
            robotSubsystem.setCoastMode();
        }

        if (armController.getRawButton(Constants.X_Button)) {
            limelight.scoreCones(robotSubsystem, arm, claw);
        }
        else if (armController.getRawButton(Constants.A_Button)) {
            limelight.pickup(robotSubsystem, arm, claw, lightSensor, true);
        }  
        else if (armController.getRawButton(Constants.B_Button)) {
            limelight.pickup(robotSubsystem, arm, claw, lightSensor, false);
        }

        else { // no limelight commands`
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
        if (highTargetTimer.get() >= 0.25) {
            arm.highTarget2();
            highTargetTimer.stop();
            highTargetTimer.reset();
        }
        if (sliderTimer.get() >= 0.7) { // redo the timing for this
            arm.sliderTarget2();
            sliderTimer.stop();
            sliderTimer.reset();
        } 
        if (defaultTimer.get() >= 0.3) {
            arm.defaultTarget2();
            defaultTimer.stop();
            defaultTimer.reset();
        } 
        


        if (driveController.getRawButtonPressed(Constants.R_joystick_Button) || armController.getRawButtonPressed(Constants.X_Button)) {
            arm.defaultTarget1();
            defaultTimer.start();
        }
        else if(driveController.getRawButtonPressed(Constants.L_joystick_Button) || armController.getRawButtonPressed(Constants.A_Button)) {
            arm.pickupTarget();
        }
        else if (driveController.getRawButtonPressed(Constants.Start_Button) || armController.getRawButtonPressed(Constants.Y_Button)) {
            arm.highTarget1();
            highTargetTimer.start();
        }
        else if(driveController.getRawButtonPressed(Constants.Y_Button) || armController.getRawButtonPressed(Constants.Back_Button)){
            arm.sliderTarget1();
            sliderTimer.start();
        }
        else if(driveController.getRawButtonPressed(Constants.Back_Button) || armController.getRawButtonPressed(Constants.B_Button)) {
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