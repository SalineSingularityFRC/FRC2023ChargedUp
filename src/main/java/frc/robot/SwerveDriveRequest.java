package frc.robot;

/*
* This class represents a request that some other code is sending to this class to tell the swerve module
* how to drive.
*/

public class SwerveDriveRequest {
    public double velocity; // Velocity: Speed for the module to go from 0 (stopped) to 1.0 (full speed)
    public double direction; // Direction: Angle (in radians) for the module to point towards while driving (robot-centric)

    public SwerveDriveRequest(double velocity, double direction) {
        this.velocity = velocity;
        this.direction = direction;
    }
}
