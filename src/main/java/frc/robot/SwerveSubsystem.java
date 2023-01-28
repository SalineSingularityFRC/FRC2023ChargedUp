package frc.robot;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.DumbNavXClasses.NavX;

/*
 * This class provides functions to drive at a given angle and direction,
 * and performs the calculations required to achieve that
 */
public class SwerveSubsystem {
// public class SwerveSubsystem implements UpdateManager.Updatable {
    /*
     * This class should own the pidgeon 2.0 IMU gyroscope that we will be using and
     * a dictionary that will house
     * all of our SwerveModules
     */
    private SwerveModule[] swerveModules = new SwerveModule[4];
    private NavX gyro;

    private final int FL = 0;
    private final int FR = 1;
    private final int BL = 2;
    private final int BR = 3;

    private final Vector[] vectorKinematics = new Vector[4];
    private final SwerveKinematics swerveKinematics;

    private SwerveDriveRequest driveSignal = null;

    /*
     * This constructor should create an instance of the pidgeon class, and should
     * construct four copies of the
     * SwerveModule class and add them to our SwerveModule dictionary
     * Use values from the Constants.java class
     */
    public SwerveSubsystem() {
        gyro = new NavX(Port.kMXP);
        // gyro = new Pigeon2(Constants.GYRO_CANCODER_ID, Constants.CANBUS);

        vectorKinematics[FL] = new Vector(Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0);
        vectorKinematics[FR] = new Vector(Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0);    
        vectorKinematics[BL] = new Vector(-Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0);   
        vectorKinematics[BR] = new Vector(-Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0);

        swerveKinematics = new SwerveKinematics(vectorKinematics);

        swerveModules[FL] = new SwerveModule(Constants.FL_Motor_ID, Constants.FL_ANGLE_ID, Constants.FL_CANCODER_ID, Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET, Constants.CANIVORE, Constants.FL_isInverted);
        swerveModules[FR] = new SwerveModule(Constants.FR_Motor_ID, Constants.FR_ANGLE_ID, Constants.FR_CANCODER_ID, Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET, Constants.CANIVORE, Constants.FR_isInverted);
        swerveModules[BL] = new SwerveModule(Constants.BL_Motor_ID, Constants.BL_ANGLE_ID, Constants.BL_CANCODER_ID, Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET, Constants.CANIVORE, Constants.BL_isInverted);
        swerveModules[BR] = new SwerveModule(Constants.BR_Motor_ID, Constants.BR_ANGLE_ID, Constants.BR_CANCODER_ID, Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET, Constants.CANIVORE, Constants.BR_isInverted);
    }

    public static class SwerveRequest {
        public double rotation;
        public Vector movement;

        public SwerveRequest(double rotation, double x, double y) {
            this.rotation = rotation;
            this.movement = new Vector(x, y);
        }
    }

    public void drive(Vector vector, double angularVelocity) { // FIGURE UOT WAY FOR ANGULAR VELOCITY
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(new Vector(0, 0), 0.0);
        }
        else {
            chassisVelocity = new ChassisVelocity(
                    vector,
                    angularVelocity
            );
        }

        Vector[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity); // needs to be dictionary
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for (int i = 0; i < moduleOutputs.length; i++) {
            SwerveModule module = swerveModules[i];
            SwerveDriveRequest request = driveInstructions(moduleOutputs[i]); // add a dictionary here
            module.drive(request);
            //SmartDashboard.putNumber(Integer.toString(i), module.getTargetVelocity());
        }
    }

    /*
     * This method takes a field-centric 
     * direction vector for
     * which way the module should travel and outputs the instruction for the individual module
     */
    public SwerveDriveRequest driveInstructions(Vector vector) { 
        double x = vector.x;
        double y = vector.y;

        double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double angle;

        if (y == 0) { // y = 0 wouldn't work because fraction
            if (x > 0) {
                angle = (3 * Math.PI) / 2;
            } 
            else if (x < 0) {
                angle = Math.PI / 2;
            } 
            else { // x = 0
                angle = -1;
            }
        }
        else if (y < 0 || (x == 0 && y < 0)) { // Q3 and Q4 and south field centric
            angle = Math.PI - Math.atan(x / y);
        }
        else if (x > 0 && y > 0) { // Q1 or north field centric
            angle = 2 * Math.PI - Math.atan(x / y);
        }
        else if (x <= 0 && y > 0) { // Q2 or north field centric
            angle = -1 * Math.atan(x / y);
        }
        else { // this else statement is useful as a catch all
            angle = 0;
        }

        if (angle != -1) {
            angle -= this.getRobotAngle() % (2 * Math.PI);

            // if (angle > 0) { // if angle postive
            //     angle += this.getRobotAngle() % 360;
            // }
            // else { // if angle is 0
            //     angle = this.getRobotAngle() - angle;
            // }
    
            // if (angle > 360) { // the setAngle class in SwerveAngle wants [0,360]
            //     angle = 360 - angle;
            // }
    
            // add recalculating the angle based of field centric view
    
            return new SwerveDriveRequest(speed, angle);
        }
        else {
            return new SwerveDriveRequest(0, getRobotAngle());
        }
    }
    
    /*
     * This function returns the angle (in radians) of the robot based on the value
     * from the pidgeon 2.0
     */
    public double getRobotAngle() {
        return ((360 - gyro.getAngle().toDegrees()) * Math.PI) / 180; // returns in counterclockwise hence why 360 minus
        // return ((360 - gyro.getAngle()) * Math.PI) / 180; // returns in counterclockwise hence why 360 minus
    }

    // public void resetGyro() {
    //     gyro.reset();
    // }
}
