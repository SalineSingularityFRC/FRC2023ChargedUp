package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {

    private double x;
    private double y;
    private double area;

    private NetworkTable table;
    private NetworkTableEntry tx, ty, ta, ledMode, camMode, pipeLine;

    //TODO Find limelight distance value
    public double target_distance = 0.0;


    public Limelight() {

        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        //state of LEDs: 1.0 - on, 2.0 - blink, 3.0 - off
        camMode = table.getEntry("camMode");

        // swap the limelight between vision processing (0) and drive camera (1)
        ledMode = table.getEntry("ledMode");

        //set pipeLine
        pipeLine = table.getEntry("pipeline");

        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    // turns on the LEDs, takes in a LimeLight object
    public void ledOn(Limelight limelight) {
        limelight.ledMode.setDouble(3.0);
    }

    // turns off the LEDs, takes in a LimeLight object
    public void ledOff(Limelight limelight) {
        limelight.ledMode.setDouble(0.0);
    }
    
    //method to switch camera between drive mode and vision mode
    public void setCamMode(Limelight limelight, double mode){
        limelight.camMode.setDouble(mode); 
    }

    // method to change between pipeLines, takes an int and a LimeLight object
    public void setpipeline(Limelight limelight, int pipe){
        limelight.pipeLine.setNumber(pipe);
    }

    /**
     * tv Whether the limelight has any valid targets (0 or 1)
     */
    public boolean getIsTargetFound() {
        NetworkTableEntry tv = table.getEntry("tv");
        double v = tv.getDouble(0);
        if (v == 0.0f){
            return false;
        }else {
            return true;
        }
    }
    /**
     * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     */
    public double getdegRotationToTarget() {
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }
    
    /**
     * Uses tx to align the robot parallel to to the aprilTag (target)
     * Takes in the swerveSubsystem because need accsess to wheels
     * If the robot is aligned farther, then drive faster
     * The method first gets the degree offset to zero then drives the (not known) distance to align the arm to the target
     */
    public void alignToTarget() {
        if (getdegRotationToTarget() > 20) {
        }
    }
}
