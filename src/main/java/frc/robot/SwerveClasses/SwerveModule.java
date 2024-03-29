package frc.robot.SwerveClasses;

import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

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

  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, true, 0, 0, false);
  public MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  double[] drive_controller_gains = Constants.PidGains.SwerveModule.DRIVE_PID_CONTROLLER;
  double[] turn_controller_gains = Constants.PidGains.SwerveModule.TURNING_PID_CONTROLLER;
  private final PIDController m_drivePIDController = new PIDController(drive_controller_gains[0], drive_controller_gains[1], drive_controller_gains[2]);
  private final PIDController m_turningPIDController = new PIDController(turn_controller_gains[0], turn_controller_gains[1], turn_controller_gains[2]);

  private final double absolutePositionEncoderOffset;
  private String name;

  /*
   * This constructor needs to take two parameters, one for the CAN ID of the drive motor and one for the CAN ID of the
   * angle motor
   * It should initialize our drive motor and create a SwerveAngle, passing the CAN ID to the SwerveAngle constructor
   */
  public SwerveModule(
      int Can_ID_driveMotor,
      int Can_ID_angleMotor,
      int Can_ID_canCoder,
      double zeroPosition,
      String canNetwork,
      boolean isInverted,
      String name) { // add a zeroPosition thing
    m_encoder = new CANcoder(Can_ID_canCoder, canNetwork);
    driveMotor = new TalonFX(Can_ID_driveMotor, canNetwork);
    CurrentLimitsConfigs current = new CurrentLimitsConfigs();
    current.SupplyCurrentLimit = 30;
    current.SupplyCurrentLimitEnable = true;
    driveMotor.getConfigurator().apply(current);
    angleMotor = new SwerveAngle(Can_ID_angleMotor, canNetwork);
    this.name = name;
    driveMotor.setInverted(isInverted);
    if (isInverted) {
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    absolutePositionEncoderOffset = zeroPosition;
    this.resetZeroAngle();
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

  /*
   * This method is used in Swerve Odometry
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getEncoderPosition()));

    double driveOutput =
        m_drivePIDController.calculate(driveMotor.get(), state.speedMetersPerSecond);

    double turnOutput =
        m_turningPIDController.calculate(getEncoderPosition(), state.angle.getRadians());

    driveMotor.set(driveOutput);
    angleMotor.setAngle(state.angle.getRadians());
  }

  public double getEncoderPosition() {
    return (m_encoder.getAbsolutePosition().getValue() * 2 * Math.PI)
        - absolutePositionEncoderOffset;
  }

  public void setCoastMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(motorOutputConfigs);
  }

  public void setBrakeMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(motorOutputConfigs);
  }

  public boolean isCoast() {
    if (motorOutputConfigs.NeutralMode == NeutralModeValue.Coast) {
      return true;
    } else {
      return false; // it is brake mode
    }
  }

  public double getPosition() {
    return driveMotor.getPosition().getValue();
  }
}
