package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveRequest;

public class Limelight {
    private NetworkTable table;
    public NetworkTableEntry tx, ty, ta, tv, ledMode, camMode, pipeLine, crop;

    private double[] localization;
    private double poseX, poseY, yaw;


    public boolean isTurningDone;
    public final double minimumSpeed = 0.05;


    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        
        // swap the limelight between vision processing (0) and drive camera (1)
        camMode = table.getEntry("camMode");

        //state of LEDs: 1.0 - on, 2.0 - blink, 3.0 - off
        ledMode = table.getEntry("ledMode");

        pipeLine = table.getEntry("pipeline");


        localization = table.getEntry("botpose").getDoubleArray(new double[6]);
        poseX = localization[0];
        poseY = localization[1];
        yaw = localization[5];


        setCamMode(0); // set to vision mode
        ledOff();
        setpipeline(1);
    }

    public void runLimelight() { // to put smartdashboard values
        SmartDashboard.putNumber("tx", tx.getDouble(0.0));
        SmartDashboard.putNumber("ty", ty.getDouble(0.0));
        SmartDashboard.putNumber("ta", ta.getDouble(0.0));

        SmartDashboard.putNumber("poseX", poseX);
        SmartDashboard.putNumber("poseY", poseY);
        SmartDashboard.putNumber("yaw", yaw);
    }

    // turns on the LEDs
    public void ledOn() {
        ledMode.setDouble(3.0);
    }

    // turns off the LEDs
    public void ledOff() {
        ledMode.setDouble(0.0);
    }
    
    //method to switch camera between drive mode and vision mode
    public void setCamMode(double mode){
        camMode.setDouble(mode); 
    }

    // method to change between pipeLines, takes an int and a LimeLight object
    public void setpipeline(int pipe){
        pipeLine.setNumber(pipe);
    }

    public boolean getIsTargetFound() {
        double a = ta.getDouble(0);
        if (a <= 0.05){
            return false;
        }else {
            return true;
        }
    }

    // public boolean pickupCube(SwerveSubsystem drive) {
    //     setpipeline(2);
    //     if (!isTurningDone) {
    //         isTurningDone = turnAngle(drive);
    //     }

    //     return true;
    // }


    public boolean pickup(SwerveSubsystem drive, ArmSubsystem arm, ClawPneumatics claw, boolean isCube) {
        if (isCube) {
            setpipeline(2);
        }
        else {
            setpipeline(1); // is cone
        }

        arm.pickupTarget();

        if (claw.isClawClosed) {
            claw.setLow();
        }
        else {
            claw.setOff();
        }

        if (!isTurningDone) {
            isTurningDone = turnAngle(drive);
            return false;
        }
        else {
            // We see the target and are aimed at it, drive forwards now
            double x = 0;
            // double y = Math.abs(2.65 - ta.getDouble(0));
            double y = 0;
            if (ta.getDouble(0) <= 2.5) {
                // 0.26 is a specific multiplier that makes speed hit exactly 0.7 if ta is approx 0 (very far away)
                x = ((2.5 - ta.getDouble(0)) * 0.26) + minimumSpeed; 
                if(x > 0.7){
                    x = 0.7;
                }
            }
            // if (tx.getDouble(0) >= 9.5) {
            //     x *= -0.30;
            //     if (x < -0.5) {
            //         x = -1;
            //     }
            // }
            // else if (tx.getDouble(0) < 9.5) {
            //     x *= 0.30;
            //     if (x > 0.5) {
            //         x = 1;
            //     } 
            // }
            // else {
            //     x = 0;
            // }


            // if (tx.getDouble(0) > 10 && tx.getDouble(0) < 9 && ta.getDouble(0) < 2.5 && ta.getDouble(0) > 2.7) {
            //     claw.setLow();
            //     return true;
            // }

            if (x != 0) { // we really want the sensor for this tbh
                claw.setHigh();
                return true;
            }

            drive.drive(new SwerveRequest(0, x, y), false);
            return false;
        }
    }
    
    public boolean turnAngle(SwerveSubsystem drive){
        if (getIsTargetFound()) {

            double speed = 0;
            if (tx.getDouble(0) >= 9.5) {
                // 0.0275 is a specific multiplier that makes speed hit exactly 0.6 if bot is off by 20 degrees to the left
                speed = ((tx.getDouble(0) - 9.5) * 0.0275) + minimumSpeed; 
                if(speed > 0.6){
                    speed = 0.6;
                }
            }
            else if (tx.getDouble(0) <= 8.5) {
                // 0.0275 is a specific multiplier that makes speed hit exactly -0.6 if bot is off by 20 degrees to the right
                speed = ((8.5 - tx.getDouble(0)) * 0.0275) + minimumSpeed; 
                if(speed > 0.6){
                    speed = 0.6;
                }
    
                speed *= -1; // because we want to turn clockwise here
            }
            else { 
                drive.drive(new SwerveRequest(0 , 0, 0), false);
                return true; // we are angled correctly
            }
    
            drive.drive(new SwerveRequest(speed, 0, 0), false);
            return false;

        }
        else {
            return false;
            // did not see anything
        }
    }



    // psuedo code
    // public boolean scoreCubes(SwerveSubsystem drive, ArmSubsystem arm, ClawPneumatics claw, boolean isHighTarget) {

    // } 
}
