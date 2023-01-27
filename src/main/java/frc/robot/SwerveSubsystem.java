package frc.robot;

import java.util.Hashtable;
import java.util.Map;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.UpdateManager.Updatable;

/*
 * This class provides functions to drive at a given angle and direction,
 * and performs the calculations required to achieve that
 */
public class SwerveSubsystem implements UpdateManager.Updatable {
    /*
     * This class should own the pidgeon 2.0 IMU gyroscope that we will be using and
     * a dictionary that will house
     * all of our SwerveModules
     */
    private Map<String, SwerveModule> swerveModules = new Hashtable<String, SwerveModule>();
    private Pigeon2 gyro;

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector(Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0),         // Front left
            new Vector(Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0),        // Front right
            new Vector(-Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0),        // Back left
            new Vector(-Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0)        // Back right
    );

    private final Object stateLock = new Object();
    // @GuardedBy("stateLock")

    private SwerveDriveRequest driveSignal = null;

    // we'll cross this bridge when we get to it

    /*
     * This constructor should create an instance of the pidgeon class, and should
     * construct four copies of the
     * SwerveModule class and add them to our SwerveModule dictionary
     * Use values from the Constants.java class
     */
    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.GYRO_CANCODER_ID, Constants.CANBUS);

        swerveModules.put("FL", new SwerveModule(Constants.FL_Motor_ID, Constants.FL_ANGLE_ID, Constants.FL_CANCODER_ID, Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET, Constants.CANIVORE, Constants.FL_isInverted));
        swerveModules.put("FR", new SwerveModule(Constants.FR_Motor_ID, Constants.FR_ANGLE_ID, Constants.FR_CANCODER_ID, Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET, Constants.CANIVORE, Constants.FR_isInverted));
        swerveModules.put("BL", new SwerveModule(Constants.BL_Motor_ID, Constants.BL_ANGLE_ID, Constants.BL_CANCODER_ID, Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET, Constants.CANIVORE, Constants.BL_isInverted));
        swerveModules.put("BR", new SwerveModule(Constants.BR_Motor_ID, Constants.BR_ANGLE_ID, Constants.BR_CANCODER_ID, Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET, Constants.CANIVORE, Constants.BR_isInverted));
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
        for (String key : swerveModules.keySet()) {
            SwerveModule module = swerveModules.get(key);
            SwerveDriveRequest request = drive(module.output[i]); // add a dictionary here
            module.drive(moduleOutputs[], key);
            //SmartDashboard.putNumber(Integer.toString(i), module.getTargetVelocity());
        }
    }

    /*
     * This method takes a field-centric target rotation (in radians) and a
     * field-centric direction vector for
     * which way the module should travel
     */
    public SwerveDriveRequest drive(Vector vector) { // we dont use the rotation part of SwerveRequest right now
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

    // public void periodic() {
    //     for (int i = 0; i < modules.length; i++) {
    //         moduleAngleEntries[i].setDouble(Math.toDegrees(modules[i].getCurrentAngle()));
    //     }
    // }

    // @Override
    // public void update(double time, double dt) {
    //     SwerveDriveRequest driveSignal;
    //     synchronized (stateLock) {
    //         driveSignal = this.driveSignal;
    //     }

    //     updateModules(driveSignal, dt);
    // }



    
    /*
     * This function returns the angle (in radians) of the robot based on the value
     * from the pidgeon 2.0
     */
    public double getRobotAngle() {
        return ((360 - gyro.getAngle()) * Math.PI) / 180; // returns in counterclockwise hence why 360 minus
    }

    public void resetGyro() {
        gyro.reset();
    }










    /*
     * This method takes a field-centric target rotation (in radians) and a
     * field-centric direction vector for
     * which way the robot should travel
     */
    public void drive(Vector vector, String key) { // we dont use the rotation part of SwerveRequest right now
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
    
            swerveModules.get(key).drive(new SwerveDriveRequest(speed, angle));
        }
    }






    /*
     * This method takes a field-centric target rotation (in radians) and we sit
     * there and turn to it
     */
    // public void turn(double turnValue) { // between -1 and 1 for the joystick, all the way to the left is 1 and all the
    //                                      // way to the right is -1

    //     if (turnValue > 0) { // counterclockwise
    //         swerveModules.get("FL")
    //                 .drive(new SwerveModule.SwerveDriveRequest(turnValue, (0.5 * Math.PI) + Constants.theta));
    //         swerveModules.get("FR")
    //                 .drive(new SwerveModule.SwerveDriveRequest(turnValue, (0.5 * Math.PI) - Constants.theta));
    //         swerveModules.get("BL")
    //                 .drive(new SwerveModule.SwerveDriveRequest(turnValue, (1.5 * Math.PI) - Constants.theta));
    //         swerveModules.get("BR")
    //                 .drive(new SwerveModule.SwerveDriveRequest(turnValue, (1.5 * Math.PI) + Constants.theta));
    //     }

    //     else if (turnValue < 0) { // clockwise
    //         swerveModules.get("FL")
    //                 .drive(new SwerveModule.SwerveDriveRequest(Math.abs(turnValue), (0.5 * Math.PI) - Constants.theta));
    //         swerveModules.get("FR")
    //                 .drive(new SwerveModule.SwerveDriveRequest(Math.abs(turnValue), (0.5 * Math.PI) + Constants.theta));
    //         swerveModules.get("BL")
    //                 .drive(new SwerveModule.SwerveDriveRequest(Math.abs(turnValue), (1.5 * Math.PI) + Constants.theta));
    //         swerveModules.get("BR")
    //                 .drive(new SwerveModule.SwerveDriveRequest(Math.abs(turnValue), (1.5 * Math.PI) - Constants.theta));
    //     }
    // }
}
