package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveClasses.SwerveAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;


public class AutonControlScheme {

    protected ClawPneumatics clawPneumatics;
    protected SwerveSubsystem drive;
    protected String color;
    protected ArmSubsystem arm;
    protected Pigeon2 gyro;
    
    public static boolean autonEnabled = false; //Testing to see if this fixes auton code being ran in teleop

    public AutonControlScheme(ArmSubsystem arm, ClawPneumatics clawPneumatics, SwerveSubsystem drive, Pigeon2 gyro, String color){
        this.clawPneumatics = clawPneumatics;
        this.drive = drive;
        this.color = color;
        this.arm = arm;
        this.gyro = gyro;
    }

    public void driveDistance(double distance, double angle) { // angle in radians and counterclockwise
        double startingEncoderValue = drive.getSwerveModule(0).getPosition();
        double changeInEncoderValue = 0;

        double rotations;
        double startingAngle = drive.getRobotAngle();
        double x = -Math.sin(angle);
        double y = Math.cos(angle);

        while (autonEnabled && changeInEncoderValue <= distance) {
            double difference = drive.getRobotAngle() - startingAngle; 
            if (difference > 0.01) { // robot is facing left of the desired angle
                rotations = 0.1;
            }
            else if (difference < 0.01) { // robot is facing right of the desired angle
                rotations = -0.1;
            }
            else { // robot is (relatively) straight
                rotations = 0;
            }

            drive.drive(new SwerveSubsystem.SwerveRequest(rotations, x, y));
            changeInEncoderValue = Math.abs(drive.getSwerveModule(0).getPosition() - startingEncoderValue);
        }
        
        drive.drive(new SwerveSubsystem.SwerveRequest(0, 0, 0));

    }    

    public void turnAngle(double angle){ // counterclockwise

        double finalAngle = drive.getRobotAngle() + angle;
        while(autonEnabled && drive.getRobotAngle() < finalAngle){ 
            SmartDashboard.putNumber("Auton Turn Angle Robot Angle", drive.getRobotAngle());
            drive.drive(new SwerveSubsystem.SwerveRequest(0.5, 0.0, 0.0));
            // if(drive.getRobotAngle() < 0){
            //     drive.drive(new SwerveSubsystem.SwerveRequest(0.5, 0.0, 0.0), false);
            // } 
            // else if(drive.getRobotAngle() > Math.PI){
            //     drive.drive(new SwerveSubsystem.SwerveRequest(-0.5, 0.0, 0.0), false);
            // }
            
        }
        
        /*SwerveAngle bLAngle = new SwerveAngle(Constants.BL_CANCODER_ID, Constants.CANBUS);
        SwerveAngle bRAngle = new SwerveAngle(Constants.BR_CANCODER_ID, Constants.CANBUS);
        SwerveAngle fLAngle = new SwerveAngle(Constants.FL_CANCODER_ID, Constants.CANBUS);
        SwerveAngle fRAngle = new SwerveAngle(Constants.FR_CANCODER_ID, Constants.CANBUS);
        bLAngle.setAngle(angle);
        bRAngle.setAngle(angle);
        fLAngle.setAngle(angle);
        fRAngle.setAngle(angle);*/
    }

    public void getOnChargeStation() {
        double roll = gyro.getRoll().getValue();

        while (autonEnabled) {
            if (roll > 10) {
                driveDistance(5, Math.PI); // drive backwards
            }
            else if (roll < -10) {
                driveDistance(5, 0); // drive forward
            }
            else {
                break;
            }
        }

        // if (roll > 10) {
        //     driveDistance(5, Math.PI); // drive backwards
        // }
        // else if (roll < -10) {
        //     driveDistance(5, 0); // drive forward
        // }
        // else {
        //     return true;
        // }

        // return false;
    }

    public void setLowerClawPreset() {
        arm.pickupTarget();
    }

    public void setHighClawPreset() {
        arm.highTarget();
        clawPneumatics.setLow();
    }
}