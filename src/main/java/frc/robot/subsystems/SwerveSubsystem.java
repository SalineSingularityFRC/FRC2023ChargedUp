package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;

// import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.SwerveClasses.ChassisVelocity;
import frc.robot.SwerveClasses.SwerveDriveRequest;
import frc.robot.SwerveClasses.SwerveKinematics;
import frc.robot.SwerveClasses.SwerveModule;
import frc.robot.SwerveClasses.Vector;

// import com.kauailabs.navx.frc.AHRS;
// import frc.robot.DumbNavXClasses.NavX;


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
    public Pigeon2 gyro;

    private final int FL = 0;
    private final int FR = 1;
    private final int BL = 2;
    private final int BR = 3;

    private SwerveModule[] swerveModules = new SwerveModule[4];
    private final Vector[] vectorKinematics = new Vector[4];
    private final SwerveKinematics swerveKinematics;

    /*
     * This constructor should create an instance of the pidgeon class, and should
     * construct four copies of the
     * SwerveModule class and add them to our SwerveModule dictionary
     * Use values from the Constants.java class
     */
    public SwerveSubsystem() {
        // gyro = new NavX(Port.kMXP);
        gyro = new Pigeon2(Constants.GYRO_CANCODER_ID, Constants.CANIVORE);
        
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

    public static class SwerveRequest { // this class represents what our controller is telling our robot to do
        public double rotation;
        public Vector movement;

        public SwerveRequest(double rotation, double x, double y) {
            this.rotation = rotation;
            this.movement = new Vector(x, y);
        }
    }

    public void drive(SwerveRequest swerveRequest, boolean isConstantMode) { // takes in the inputs from the controller
        ChassisVelocity chassisVelocity;
        boolean isMoving = true;
        
        SmartDashboard.putNumber("x", swerveRequest.movement.x);
        SmartDashboard.putNumber("y", swerveRequest.movement.y);
        SmartDashboard.putNumber("rotation", swerveRequest.rotation);

        // vector direction and turning is not changed, but speed is reduced to a constant 
        if (isConstantMode) {
            if (swerveRequest.rotation > 0) {
                swerveRequest.rotation = Constants.SNAIL_SPEED;
            }
            else if (swerveRequest.rotation < 0) {
                swerveRequest.rotation = -Constants.SNAIL_SPEED;
            }

            double divisor = Constants.SNAIL_SPEED / (Math.pow((Math.pow(swerveRequest.movement.x, 2) * Math.pow(swerveRequest.movement.y, 2)), 0.5));
            swerveRequest.movement.x *= divisor;
            swerveRequest.movement.y *= divisor;
        }

        // this is to make sure if both the joysticks are at neutral position, the robot and wheels don't move or turn at all
        // 0.05 value can be increased if the joystick is increasingly inaccurate at neutral position
        if (Math.abs(swerveRequest.movement.x) < 0.05 
            && Math.abs(swerveRequest.movement.y) < 0.05 
            && Math.abs(swerveRequest.rotation) < 0.05) {
            for (int i = 0; i < swerveModules.length; i++) {
                swerveModules[i].coast();
            }
            return;
        }
        else {
            chassisVelocity = new ChassisVelocity(swerveRequest.movement, swerveRequest.rotation); 

            if (Math.abs(swerveRequest.movement.x) < 0.05 
            && Math.abs(swerveRequest.movement.y) < 0.05) {
                isMoving = false; // this boolean is to ensure that gyro wont be used if it is just turning
            }
        }

        Vector[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity); 
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1); // these two lines are what calculates the module angles for swerve
        for (int i = 0; i < moduleOutputs.length; i++) {
            SwerveModule module = swerveModules[i];
            SwerveDriveRequest request;

            // String string = "wheel #" + i;

            // These if else are to swap the moduleOutputs for the FR and BL modules
            // TO-DO research on why we need this, but for now, it works as it is so its fine
            if (i == 1) {
                i = 2;
                request = driveInstructions(moduleOutputs[i], isMoving);
                i = 1;
            } else if (i == 2) {
                i = 1;
                request = driveInstructions(moduleOutputs[i], isMoving);
                i = 2;
            } else {
                request = driveInstructions(moduleOutputs[i], isMoving); 
                /* 
                this method is used to convert the vector that the swerveKinematics outputs for each wheel
                into direction and speed instructions that the module.drive method can take in
                */
            }

            if (request.direction != 0) {
                request.direction = (2 * Math.PI) - request.direction; 
                // IMPORTANT: this bit is to convert the direction from clockwise to counterclockwise
                // Their kinematics class outputs clockwise degree, but our methods take in a counterclockwise degree
            } 

            module.drive(request);
        }
    }

    /*
     * This method takes a field-centric 
     * direction vector for
     * which way the module should travel and outputs the SwerveDriveRequest instruction for the individual module
     */
    public SwerveDriveRequest driveInstructions(Vector vector, boolean isMoving) { 
        double x = vector.x;
        double y = vector.y;

        double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) / Constants.SPEED_DIVISOR; // speed kills
        double angle;

        if (y == 0) { // y = 0 wouldn't work because fraction
            if (x > 0) {
                angle = (3 * Math.PI) / 2;
            } 
            else if (x < 0) {
                angle = Math.PI / 2;
            } 
            else { // x = 0
                return new SwerveDriveRequest(0, 0); // this should never happen
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

        if (isMoving) { // if false, then it is only turning and does not want to be affected by the gyro
            angle += this.getRobotAngle() % (2 * Math.PI); // this is to make it field centric
        }
        return new SwerveDriveRequest(speed, angle);
    }
    
    /*
     * This function returns the angle (in radians) of the robot based on the value
     * from the pidgeon 2.0
     */
    public double getRobotAngle() {
        //return ((360 - gyro.getAngle().toDegrees()) * Math.PI) / 180; // for NavX
        return ((360 - (gyro.getAngle() - 180)) * Math.PI) / 180; // returns in counterclockwise hence why 360 minus
        // it is gyro.getAngle() - 90 because the pigeon for this robot is facing west (north is forward)
    }

    public void resetGyro() {
        gyro.reset();
    }


    public SwerveModule getSwerveModule(int module) {
        return swerveModules[module];
    }
}
