package frc.robot;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveAngle.AnglePosition;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
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
    private SwerveAngle angleMotor;
    private CANcoder m_encoder;
    private TalonFX driveMotor;

    private final Object stateMutex = new Object();
    private double targetSpeed = 0.0;
    private double targetAngle = 0.0; // in radians

    private final double absolutePositionEncoderOffset;

    /*
     * This constructor needs to take two parameters, one for the CAN ID of the drive motor and one for the CAN ID of the
     * angle motor
     * It should initialize our drive motor and create a SwerveAngle, passing the CAN ID to the SwerveAngle constructor
     */
    public SwerveModule(int Can_ID_driveMotor, int Can_ID_angleMotor, int Can_ID_canCoder, double zeroPosition, String canNetwork, boolean isInverted) { // add a zeroPosition thing
        m_encoder = new CANcoder(Can_ID_canCoder, canNetwork);
        driveMotor = new TalonFX(Can_ID_driveMotor, canNetwork);
        angleMotor = new SwerveAngle(Can_ID_angleMotor, canNetwork);
        driveMotor.setInverted(isInverted);

        absolutePositionEncoderOffset = zeroPosition;

        this.resetZeroAngle();
    }
    

    /*
     * This function should take a swerve request and call the setAngle() method in the SwerveAngle class.
     * If the output is a AnglePosition.Positive, we should set the drive motor to the velocity request
     * If the output is a AnglePosition.Negative, we should set the drive motor to the negative velocity request
     * If the output is AnglePosition.Moving, we shoul not set the drive motor, as we are not yet angled correctly
     * This function returns true if we did set the drive motor, false if we did not 
     */

     /**
     * Sets the target velocity. The vector should have a length that is less than or equal to 1.
     *
     * @param velocity the target velocity
     */
    public final void setTargetVelocity(Vector2 velocity) {
        synchronized (stateMutex) {
            targetSpeed = velocity.length;
            targetAngle = velocity.getAngle().toRadians();
        }
    }

    public final void setTargetVelocity(double speed, double angle) {
        if (speed < 0.0) {
            speed *= -1.0;

            angle += Math.PI;
        }

        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        synchronized (stateMutex) {
            targetSpeed = speed;
            targetAngle = angle;
        }
    }


    public boolean drive(SwerveDriveRequest request) {
        SwerveAngle.AnglePosition angle = angleMotor.setAngle(request.direction);
        if (angle == SwerveAngle.AnglePosition.Positive) {
           driveMotor.set(request.velocity);
            return true;
        }

        if (angle == SwerveAngle.AnglePosition.Negative) {
           driveMotor.set(-request.velocity);
            return true;
        }
        
        driveMotor.set(0);
        return false;
    }
    
    /*
     * Set the zero angle based on the current angle (in radians) that we are reading from an external source(absolute encoder).
     * We should be able to read the current angle from the CANcoder and pass that to the setZeroAngle method in
     * the SwerveAngle class
     * The can is where we base all of our correct angles on. 
     * The talon says we are at an angle but sometimes that might not be the right angle. 
     * The zeroAngle is what we use to offset(balance) whatever we're reading off the talon
     */
    public void resetZeroAngle() {
        angleMotor.setZeroAngle((m_encoder.getAbsolutePosition().getValue() * 2 * Math.PI) - absolutePositionEncoderOffset);
    }
}
