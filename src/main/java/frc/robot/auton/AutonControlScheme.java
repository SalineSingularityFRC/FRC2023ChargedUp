package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.SwerveClasses.SwerveAngle;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;


public class AutonControlScheme {

    protected ClawPneumatics clawPneumatics;
    protected SwerveSubsystem drive;
    protected String color;

    private double targetAngle;

    public AutonControlScheme(ClawPneumatics clawPneumatics, SwerveSubsystem drive, String color){
        this.clawPneumatics = clawPneumatics;
        this.drive = drive;
        this.color = color;

        targetAngle = 0;
        // targetAngle = 180;
    }

    public void driveDistance(double distance, double angle) { // angle in radians and counterclockwise
        double startingEncoderValue = drive.getSwerveModule(0).getPosition();
        double changeInEncoderValue = 0;

        double rotations;
        targetAngle = angle;
        double x = -Math.sin(angle);
        double y = Math.cos(angle);

        while (changeInEncoderValue <= distance) {
            double difference = drive.getRobotAngle() - targetAngle; 
            if (difference > 0.01) { // robot is facing left of the desired angle
                rotations = -0.1;
            }
            else if (difference < 0.01) { // robot is facing right of the desired angle
                rotations = 0.1;
            }
            else { // robot is (relatively) straight
                rotations = 0;
            }

            drive.drive(new SwerveSubsystem.SwerveRequest(0, x, y), false);
            changeInEncoderValue = Math.abs(drive.getSwerveModule(0).getPosition() - startingEncoderValue);
        }
        
        drive.drive(new SwerveSubsystem.SwerveRequest(0, 0, 0), false);

    }    

    public void turnAngle(double angle){

        //TODO: Change angle from 45 to something else
        angle = Math.PI / 4;
        SwerveAngle bLAngle = new SwerveAngle(Constants.BL_CANCODER_ID, Constants.CANBUS);
        SwerveAngle bRAngle = new SwerveAngle(Constants.BR_CANCODER_ID, Constants.CANBUS);
        SwerveAngle fLAngle = new SwerveAngle(Constants.FL_CANCODER_ID, Constants.CANBUS);
        SwerveAngle fRAngle = new SwerveAngle(Constants.FR_CANCODER_ID, Constants.CANBUS);
        bLAngle.setAngle(angle);
        bRAngle.setAngle(angle);
        fLAngle.setAngle(angle);
        fRAngle.setAngle(angle);
    }
}