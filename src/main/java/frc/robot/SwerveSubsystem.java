package frc.robot;

import com.ctre.phoenixpro.hardware.Pigeon2;

// import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private SwerveModule[] swerveModules = new SwerveModule[4];
    private Pigeon2 gyro;

    private final int FL = 0;
    private final int FR = 1;
    private final int BL = 2;
    private final int BR = 3;

    private final Vector[] vectorKinematics = new Vector[4];
    private final SwerveKinematics swerveKinematics;

    private double targetAngle;
    /*
     * This constructor should create an instance of the pidgeon class, and should
     * construct four copies of the
     * SwerveModule class and add them to our SwerveModule dictionary
     * Use values from the Constants.java class
     */
    public SwerveSubsystem() {
        //gyro = new NavX(Port.kMXP);
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

    public static class SwerveRequest {
        public double rotation;
        public Vector movement;

        public SwerveRequest(double rotation, double x, double y) {
            this.rotation = rotation;
            this.movement = new Vector(x, y);
        }
    }

    public void drive(SwerveRequest swerveRequest) { 
        ChassisVelocity chassisVelocity;
        boolean isMoving = true;
        // if (driveSignal == null) {
        //     chassisVelocity = new ChassisVelocity(new Vector(0, 0), 0.0);
        // }
        SmartDashboard.putNumber("GYRO", getRobotAngle());
        SmartDashboard.putNumber("x", swerveRequest.movement.x);
        SmartDashboard.putNumber("y", swerveRequest.movement.y);
        SmartDashboard.putNumber("rotation", swerveRequest.rotation);

        // this is to make sure if both the joysticks are at neutral position, the robot and wheels don't move or turn at all
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
                isMoving = false;
            }
        }

        Vector[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity); 
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for (int i = 0; i < moduleOutputs.length; i++) {
            SwerveModule module = swerveModules[i];
            SwerveDriveRequest request;

            String string = "wheel #" + i;

            SmartDashboard.putNumber(string + " VECTOR X", moduleOutputs[i].x);
            SmartDashboard.putNumber(string + " VECTOR Y", moduleOutputs[i].y);

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
            }

            if (request.direction != 0) {
                request.direction = (2 * Math.PI) - request.direction; 
                // IMPORTANT: this bit is to convert the direction from clockwise to counterclockwise
                // Their kinematics class outputs clockwise degree, but our methods take in a counterclockwise degree
            } 

            SmartDashboard.putNumber(string, module.getEncoderPosition());

            SmartDashboard.putNumber(string + " direction", request.direction);
            module.drive(request);
            //SmartDashboard.putNumber(Integer.toString(i), module.getTargetVelocity());
        }
    }

    /*
     * This method takes a field-centric 
     * direction vector for
     * which way the module should travel and outputs the instruction for the individual module
     */
    public SwerveDriveRequest driveInstructions(Vector vector, boolean isMoving) { 
        double x = vector.x;
        double y = vector.y;

        double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))/4; // speed kills
        double angle;

        if (y == 0) { // y = 0 wouldn't work because fraction
            if (x > 0) {
                angle = (3 * Math.PI) / 2;
            } 
            else if (x < 0) {
                angle = Math.PI / 2;
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

        if (isMoving) {
            angle += this.getRobotAngle() % (2 * Math.PI); // this is to make it field centric
        }
        return new SwerveDriveRequest(speed, angle);
        // if (angle != -1) {
        //     if (isMoving) {
        //         angle += this.getRobotAngle() % (2 * Math.PI); // this is to make it field centric
        //     }
        //     return new SwerveDriveRequest(speed, angle);
        // }
        // else {
        //     SmartDashboard.putNumber("the very real angle", getRobotAngle());

        //     return new SwerveDriveRequest(0, );
        // }
    }
    
    /*
     * This function returns the angle (in radians) of the robot based on the value
     * from the pidgeon 2.0
     */
    public double getRobotAngle() {
        //return ((360 - gyro.getAngle().toDegrees()) * Math.PI) / 180; // for NavX
        return ((360 - gyro.getAngle()) * Math.PI) / 180; // returns in counterclockwise hence why 360 minus
    }

    public void resetGyro() {
        gyro.reset();
    }
}
