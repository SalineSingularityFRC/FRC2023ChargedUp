package frc.robot;

/*
 * This class owns the components of a single swerve module and is responsible for controlling
 * the angle and speed that the module is moving
 */
public class SwerveModule {
    /*
     * We will need a couple different instance variables
     *   An instance of the SwerveAngle class to handle the angle motor
     *   An instance of the TalonFX class to handle the drive motor
     *   An instance of the CANcoder class to handle the encoder
     */

    /*
     * This constructor needs to take two parameters, one for the CAN ID of the drive motor and one for the CAN ID of the
     * angle motor
     * It should initialize our drive motor and create a SwerveAngle, passing the CAN ID to the SwerveAngle constructor
     */
    public SwerveModule() {
        throw new UnsupportedOperationException();
    }

    /*
     * This class represents a request that some other code is sending to this class to tell the swerve module
     * how to drive.
     * Velocity: Speed for the module to go from 0 (stopped) to 1.0 (full speed)
     * Direction: Angle (in radians) for the module to point twards while driving (robot-centric)
     */
    public class SwerveDriveRequest {
        public double velocity;
        public double direction;
    }

    /*
     * This function should take a swerve request and call the setAngle() method in the SwerveAngle class.
     * If the output is a AnglePosition.Positive, we should set the drive motor to the velocity request
     * If the output is a AnglePosition.Negative, we should set the drive motor to the negative velocity request
     * If the output is AnglePosition.Moving, we shoul not set the drive motor, as we are not yet angled correctly
     * This function returns true if we did set the drive motor, false if we did not 
     */
    public boolean drive(SwerveDriveRequest request) {
        throw new UnsupportedOperationException();
    }
    
    /*
     * Set the zero angle based on the current angle (in radians) that we are reading from an external source.
     * We should be able to read the current angle from the CANcoder and pass that to the setZeroAngle method in
     * the SwerveAngle class
     */
    public void resetZeroAngle() {

    }
}
