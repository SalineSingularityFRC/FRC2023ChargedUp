package frc.robot;

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

    //TODO Find limelight distance value
    public double target_distance = 0.0;


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

        //set pipeLine
        pipeLine = table.getEntry("pipeline");


        setCamMode(0);
        ledOff();
        setpipeline(1);
    }

    public void runLimelight() {
        SmartDashboard.putNumber("tx", tx.getDouble(0.0));
        SmartDashboard.putNumber("ty", ty.getDouble(0.0));
        SmartDashboard.putNumber("ta", ta.getDouble(0.0));
        SmartDashboard.putNumber("tv", tv.getDouble(0.0));
    }

    // turns on the LEDs, takes in a LimeLight object
    public void ledOn() {
        ledMode.setDouble(3.0);
    }

    // turns off the LEDs, takes in a LimeLight object
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

    /**
     * tv Whether the limelight has any valid targets (0 or 1)
     */
    // public boolean getIsTargetFound() {
    //     double v = tv.getDouble(0);
    //     if (v == 0.0){
    //         return false;
    //     }else {
    //         return true;
    //     }
    // }

    public boolean getIsTargetFound() {
        double a = ta.getDouble(0);
        if (a <= 0.05){
            return false;
        }else {
            return true;
        }
    }

    public boolean pickupCube(SwerveSubsystem drive, ArmSubsystem arm, ClawPneumatics claw) {
        setpipeline(2);
        arm.pickupTarget();
        if (claw.isClawClosed) {
            claw.setLow();
        }
        else {
            claw.setOff();
        }

        double rotation = 0;
        double x = Math.abs(9.5 - ta.getDouble(0));
        double y = Math.abs(2.65-ta.getDouble(0));
        if (getIsTargetFound()) {
            // We do see the target, execute aiming code
            // if (drive.getRobotAngle() % (Math.PI * 2) > 0.1) {
            //     rotation = -0.05;
            // }
            // else if (drive.getRobotAngle() % (Math.PI * 2) < 0.1) {
            //     rotation = 0.05;
            // }
            // else {
            //     rotation = 0;
            // }


            if (ta.getDouble(0) > 2.7) {
                y *= -0.15;
            }
            else if (ta.getDouble(0) < 2.6) {
                y *= 0.15;
            }
            else {
                y = 0;
            }
            if (y > 0.5) {
                y = 0.5;
            } 
            else if (y < -0.5) {
                y = -0.5;
            }

            if (tx.getDouble(0) > 10) {
                x = -0.02;
            }
            else if (tx.getDouble(0) < 9) {
                x = 0.02;
            }
            else {
                x = 0;
            }
            if (x > 0.5) {
                x = 0.5;
            } 
            else if (x < -0.5) {
                x = -0.5;
            }

            if (rotation == 0 && x == 0 && y == 0) {
                claw.setHigh();
                return true;
            }

            drive.drive(new SwerveRequest(rotation, x, y), false);
            return true;
        }
        else {
            return false;
            // did not see anything
        }
    }
    
    /**
     * Uses tx to align the robot parallel to to the aprilTag (target)
     * Takes in the swerveSubsystem because need accsess to wheels
     * If the robot is aligned farther, then drive faster
     * The method first gets the degree offset to zero then drives the (not known) distance to align the arm to the target
     */
    public void alignToTarget() {
        if (tx.getDouble(0.0) > 20) {
        }
    }
}
