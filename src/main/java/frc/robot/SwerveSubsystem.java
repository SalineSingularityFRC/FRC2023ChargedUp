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

    }

    /*
     * This function returns the angle (in radians) of the robot based on the value from the pidgeon 2.0
     */
    public double getRobotAngle() {
        return  
        
        (gyro.getAngle() * Math.PI) / 180;
    }

    public void resetGyro() {
        gyro.reset();
    }
}
