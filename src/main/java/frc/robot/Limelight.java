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
}
