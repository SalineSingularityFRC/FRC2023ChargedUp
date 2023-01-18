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
     * This class should own the pidgeon 2.0 IMU gyroscope that we will be using and
     * a dictionary that will house
     * all of our SwerveModules
     */
    private Dictionary<String, SwerveModule> swerveModules = new Hashtable<String, SwerveModule>();
    private Pigeon2 gyro;

    /*
     * This constructor should create an instance of the pidgeon class, and should
     * construct four copies of the
     * SwerveModule class and add them to our SwerveModule dictionary
     * Use values from the Constants.java class
     */
    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.GYRO_CANCODER_ID, Constants.CANBUS);

        swerveModules.put("FL", new SwerveModule(Constants.FL_Motor_ID, Constants.FL_ANGLE_ID, Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET));
        swerveModules.put("FR", new SwerveModule(Constants.FR_Motor_ID, Constants.FR_ANGLE_ID, Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET));
        swerveModules.put("BL", new SwerveModule(Constants.BL_Motor_ID, Constants.BL_ANGLE_ID, Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET));
        swerveModules.put("BR", new SwerveModule(Constants.BR_Motor_ID, Constants.BR_ANGLE_ID, Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET));
    }

    public class SwerveRequest {
        public double rotation;
        public Vector movement;
    }

    /*
     * This method takes a field-centric target rotation (in radians) and a
     * field-centric direction vector for
     * which way the robot should travel
     */
    public void drive(SwerveRequest request) {
        double x = request.movement.x;
        double y = request.movement.y;

        double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double angle;

        if (y == 0) { // y = 0 wouldn't work because fraction
            if (x > 0) {
                angle = 270;
            } 
            else if (x < 0) {
                angle = 90;
            } 
            else { // x = 0
                angle = 0;
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

        // velocity set to 1 for now, but it can be edited to between [0, 1]
        swerveModules.get("FL").drive(new SwerveModule.SwerveDriveRequest(speed, angle));
        swerveModules.get("FR").drive(new SwerveModule.SwerveDriveRequest(speed, angle));
        swerveModules.get("BL").drive(new SwerveModule.SwerveDriveRequest(speed, angle));
        swerveModules.get("BR").drive(new SwerveModule.SwerveDriveRequest(speed, angle));

    }

    /*
     * This method takes a field-centric target rotation (in radians) and we sit
     * there and turn to it
     */
    public void turn(double turnValue) { // between -1 and 1 for the joystick, all the way to the left is 1 and all the
                                         // way to the right is -1

        if (turnValue > 0) { // counterclockwise
            swerveModules.get("FL")
                    .drive(new SwerveModule.SwerveDriveRequest(turnValue, (0.5 * Math.PI) + Constants.theta));
            swerveModules.get("FR")
                    .drive(new SwerveModule.SwerveDriveRequest(turnValue, (0.5 * Math.PI) - Constants.theta));
            swerveModules.get("BL")
                    .drive(new SwerveModule.SwerveDriveRequest(turnValue, (1.5 * Math.PI) - Constants.theta));
            swerveModules.get("BR")
                    .drive(new SwerveModule.SwerveDriveRequest(turnValue, (1.5 * Math.PI) + Constants.theta));
        }

        else if (turnValue < 0) { // clockwise
            swerveModules.get("FL")
                    .drive(new SwerveModule.SwerveDriveRequest(Math.abs(turnValue), (0.5 * Math.PI) - Constants.theta));
            swerveModules.get("FR")
                    .drive(new SwerveModule.SwerveDriveRequest(Math.abs(turnValue), (0.5 * Math.PI) + Constants.theta));
            swerveModules.get("BL")
                    .drive(new SwerveModule.SwerveDriveRequest(Math.abs(turnValue), (1.5 * Math.PI) + Constants.theta));
            swerveModules.get("BR")
                    .drive(new SwerveModule.SwerveDriveRequest(Math.abs(turnValue), (1.5 * Math.PI) - Constants.theta));
        }
    }

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
}
