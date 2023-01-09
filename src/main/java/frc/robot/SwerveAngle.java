package frc.robot;

/**
 * This class owns a single Swerve Module's angle motor and is responsible for driving that motor to a given angle
 */
public class SwerveAngle {
    /*
     * We need to have two class variables, a Falcon motor that we can use to control the angle of the module
     * and a variable for the zero poisition of the motor in radians (what value we need for it to be straight forward)
     */

    /* 
     * Our constructor needs to take a parameter that determines which CAN ID the falcon we are using has 
     * and it needs to initialize the falcon motor and configure it (things like PID values and such)
     */
    public SwerveAngle() {
        throw new UnsupportedOperationException();
    }
     
    /*
     * This enum provides the current state of the angle motor, we should either be matching the requested angle (Positive),
     * 180° off of it (Negative), or in the process of getting to one of those positions (Moving)
     */
    public enum AnglePosition {
        Positive,
        Negative,
        Moving
    }

    /*
     * This function takes an target angle (in radians) that we want the wheel to turn to
     * and updates the target position within the falcon motor so that it will move there.
     * It returns a value that indicates if we are in the target position, or 180° off of it,
     * or still working to get to one of those positions
     */
    public AnglePosition setAngle(double targetAngle) {
        throw new UnsupportedOperationException();
    }

    /*
     * Set the zero angle based on the current angle (in radians) that we are reading from an external source.
     */
    public void setZeroAngle(double currentAngle) {

    }
}
