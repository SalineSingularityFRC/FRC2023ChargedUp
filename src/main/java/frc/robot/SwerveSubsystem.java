package frc.robot;

/*
 * This class provides functions to drive at a given angle and direction,
 * and performs the calculations required to achieve that
 */
public class SwerveSubsystem {
    /*
     * This class should own the pidgeon 2.0 IMU gyroscope that we will be using and a dictionary that will house
     * all of our SwerveModules
     */

    /*
     * This constructor should create an instance of the pidgeon class, and should construct four copies of the
     * SwerveModule class and add them to our SwerveModule dictionary
     * Use values from the Constants.java class
     */
    public SwerveSubsystem() {

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
        throw new UnsupportedOperationException();
    }
}
