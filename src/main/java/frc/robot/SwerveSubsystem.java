package frc.robot;

import java.util.Dictionary;
import java.util.Hashtable;

import com.ctre.phoenixpro.hardware.Pigeon2;

/*
 * This class provides functions to drive at a given angle and direction,
 * and performs the calculations required to achieve that
 */
public class SwerveSubsystem {
    /*
     * This class should own the pidgeon 2.0 IMU gyroscope that we will be using and a dictionary that will house
     * all of our SwerveModules
     */
    private Dictionary<String, SwerveModule> swerveModules = new Hashtable<String, SwerveModule>();
    private Pigeon2 gyro;

    /*
     * This constructor should create an instance of the pidgeon class, and should construct four copies of the
     * SwerveModule class and add them to our SwerveModule dictionary
     * Use values from the Constants.java class
     */
    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.GYRO_CANCODER_ID, Constants.CANBUS);

        swerveModules.put("FL", new SwerveModule(Constants.FL_Motor_ID, Constants.FL_ANGLE_ID));
        swerveModules.put("FR", new SwerveModule(Constants.FR_Motor_ID, Constants.FR_ANGLE_ID));
        swerveModules.put("BL", new SwerveModule(Constants.BL_Motor_ID, Constants.BL_ANGLE_ID));
        swerveModules.put("BR", new SwerveModule(Constants.BR_Motor_ID, Constants.BR_ANGLE_ID));
    }

    public class SwerveRequest {
        public double rotation;
        public Vector movement;
    }

    /*
     * This method takes a field-centric target rotation (in radians) and a field-centric direction vector for
     * which way the robot should travel
     */
    public void drive(SwerveRequest request) {
        double x = request.movement.x;
        double y = request.movement.y;

        double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double angle;

        if (x > 0 && y > 0) { // Q1
            angle = 2 * Math.PI - Math.atan(x/y);
        }

        else if (x < 0 && y > 0) { // Q2
            angle = -1 * Math.atan(x/y);
        }

        else if (y < 0) { // Q3 and Q4
            angle = Math.PI - Math.atan(x/y);
        }

        else { // this else statement is useful as a catch all 
            angle = 0;
        }

        if (angle > 0) { // if angle postive
            angle += this.getRobotAngle();
        }

        else { // if angle negative. if angle and gyro is 0
            angle = this.getRobotAngle() - angle;
        }

        // add recalculating the angle based of field centric view

        // velocity set to 1 for now, but it can be edited to between [0, 1]
        swerveModules.get("FL").drive(new SwerveModule.SwerveDriveRequest(speed, angle)); 
        swerveModules.get("FR").drive(new SwerveModule.SwerveDriveRequest(speed, angle)); 
        swerveModules.get("BL").drive(new SwerveModule.SwerveDriveRequest(speed, angle)); 
        swerveModules.get("BR").drive(new SwerveModule.SwerveDriveRequest(speed, angle)); 
        
    }

    /*
     * This method takes a field-centric target rotation (in radians) and we sit there and turn to it
     */
    public void turn(boolean isClockwise) {
        if (isClockwise) {
            swerveModules.get("FL").drive(new SwerveModule.SwerveDriveRequest(1, (0.5*Math.PI) - Constants.theta)); 
            swerveModules.get("FR").drive(new SwerveModule.SwerveDriveRequest(1, (0.5*Math.PI) + Constants.theta)); 
            swerveModules.get("BL").drive(new SwerveModule.SwerveDriveRequest(1, (1.5*Math.PI) + Constants.theta)); 
            swerveModules.get("BR").drive(new SwerveModule.SwerveDriveRequest(1, (1.5*Math.PI) - Constants.theta)); 
        }

        else { // switches the calculations for the FL and FR and the BL and BR
            swerveModules.get("FL").drive(new SwerveModule.SwerveDriveRequest(1, (0.5*Math.PI) + Constants.theta)); 
            swerveModules.get("FR").drive(new SwerveModule.SwerveDriveRequest(1, (0.5*Math.PI) - Constants.theta)); 
            swerveModules.get("BL").drive(new SwerveModule.SwerveDriveRequest(1, (1.5*Math.PI) - Constants.theta)); 
            swerveModules.get("BR").drive(new SwerveModule.SwerveDriveRequest(1, (1.5*Math.PI) + Constants.theta)); 
        }
    }

    /*
     * This function returns the angle (in radians) of the robot based on the value from the pidgeon 2.0
     */
    public double getRobotAngle() {
        return (gyro.getAngle() * Math.PI) / 180;
    }

    public void resetGyro() {
        gyro.reset();
    }
}
