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
    public final double minimumSpeed = 0.04;

    public Timer scoringTimer = new Timer();



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


    public boolean pickup(SwerveSubsystem drive, ArmSubsystem arm, ClawPneumatics claw, LightSensor lightSensor, boolean isCube) {
        if (isCube) {
            setpipeline(2);
        }
        else {
            setpipeline(1); // is cone
        }

        arm.pickupTarget();
        ledOff();

        if (claw.isClawClosed) {
            claw.setLow();
        }

        if (!isTurningDone) {
            isTurningDone = turnAngle(drive);
            return false;
        }
        else {
            // We see the target and are aimed at it, drive forwards now
            double x = 0;
            double y = 0;
            if (ta.getDouble(0) <= 2.5) {
                // 0.26 is a specific multiplier that makes speed hit exactly 0.7 if ta is approx 0 (very far away)
                y = ((2.5 - ta.getDouble(0)) * 0.13) + minimumSpeed; 
                if(y > 0.35){
                    y = 0.35;
                }
            }

            if (tx.getDouble(0) >= 9.5) {
                x = ((tx.getDouble(0) - 9.5) * 0.01) + minimumSpeed; 
                if (x < -0.2) {
                    x = -0.2;
                }

                x *= -1;
            }
            else if (tx.getDouble(0) < 8.5) {
                x = (8.5 - (tx.getDouble(0)) * 0.01) + minimumSpeed; 
                if (x > 0.2) {
                    x = 0.2;
                } 
            }

            if (lightSensor.isSensed()) { // we really want the sensor for this tbh
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
                // 0.005 is a specific multiplier that makes speed hit exactly 0.2 if bot is off by 20 degrees to the left
                speed = ((tx.getDouble(0) - 9.5) * 0.005) + minimumSpeed; 
                if(speed > 0.2){
                    speed = 0.2;
                }

            }
            else if (tx.getDouble(0) <= 8.5) {
                // 0.01 is a specific multiplier that makes speed hit exactly -0.2 if bot is off by 20 degrees to the right
                speed = ((8.5 - tx.getDouble(0)) * 0.01) + minimumSpeed; 
                if(speed > 0.2){
                    speed = 0.2;
                }

                speed *= -1; // because we want to turn clockwise here
            }
            else { 
                drive.drive(new SwerveRequest(0 , 0, 0), false);
                return true; // we are angled correctly
            }
    
            drive.drive(new SwerveRequest(speed, 0, 0), false);
        }
        
        return false;
    }

    public boolean scoreCones(SwerveSubsystem drive, ArmSubsystem arm, ClawPneumatics claw) {
        arm.defaultTarget();
        ledOn();
        setpipeline(4);

        double robotAngle = (drive.getRobotAngle() % (Math.PI * 2)) * (180/Math.PI); // in angles
        double x = 0;
        double y = 0;

        // 0.00061 is a specific multipler to make the rotation peak at 0.15 (including min speed) 
        double rotation = (180 - robotAngle) * 0.00061;
        if (rotation >= 0) {
            rotation += minimumSpeed;
        }
        else {
            rotation -= minimumSpeed;
        }


        if (ta.getDouble(0) <= 2.5) {
            y = ((2.5 - ta.getDouble(0)) * 0.13) + minimumSpeed; 
            if(y > 0.2){
                y = 0.2;
            }
        }

        if (tx.getDouble(0) >= 9.5) {
            x = ((tx.getDouble(0) - 9.5) * 0.0075) + minimumSpeed; 
            if (x < -0.15) {
                x = -0.15;
            }

            x *= -1;
        }
        else if (tx.getDouble(0) < 8.5) {
            x = (8.5 - (tx.getDouble(0)) * 0.0075) + minimumSpeed; 
            if (x > 0.15) {
                x = 0.15;
            } 
        }




        if (tx.getDouble(0) <= 9.5 && tx.getDouble(0) > 8.5 && ta.getDouble(0) > 2.5
                        && robotAngle <= 181 && robotAngle >= 179) { 
            scoringTimer.start();
            arm.highTarget(scoringTimer);
            if (scoringTimer.get() >= 1)  {
                claw.setLow();
                return true;
            }
        }
        else {
            drive.drive(new SwerveRequest(rotation, x, y), true); // we want field centric here
        }

        return false;
    }



    // psuedo code
    // public boolean scoreCubes(SwerveSubsystem drive, ArmSubsystem arm, ClawPneumatics claw, boolean isHighTarget) {

    // } 
}
