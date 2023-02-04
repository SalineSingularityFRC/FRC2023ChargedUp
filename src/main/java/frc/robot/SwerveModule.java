package frc.robot;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClasses.SwerveAngle;
import frc.robot.SwerveClasses.SwerveDriveRequest;

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

    public void coast() {
        driveMotor.set(0); // this is for when the joystick is not being moved at all
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
        angleMotor.setZeroAngle(getEncoderPosition());
    }

    public double getEncoderPosition() {
        return (m_encoder.getAbsolutePosition().getValue() * 2 * Math.PI) - absolutePositionEncoderOffset;
    }
}
