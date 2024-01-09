package frc.robot.commands;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class GetOnChargeStation extends CommandBase {

  protected SwerveSubsystem drive;
  protected Pigeon2 gyro;
  private boolean isFinished = false;
  private PIDController controller;

  /*
   * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
   *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
   */
  public GetOnChargeStation(SwerveSubsystem drive, Pigeon2 gyro) {
    this.drive = drive;
    this.gyro = gyro;
    double[] on_charge_station_gains = Constants.PidGains.GetOnChargeStation.GET_ON_CHARGE_STATION;
    this.controller =
        new PIDController(
            on_charge_station_gains[0], on_charge_station_gains[1], on_charge_station_gains[2]);
    this.controller.setSetpoint(0);
  }

  //    initialize() - This method sets up the command and is called immediately before the command
  // is executed for the first time and every subsequent time it is started .
  //  Any initialization code should be here.
  public void initialize() {
    drive.setBrakeMode();
  }

  /*
   *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
   *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
   */
  public void execute() {
    double pitch = gyro.getRoll().getValue();
    double speed = Math.abs(pitch / 14);

    if (pitch > 2.3) {
      drive.drive(new SwerveSubsystem.SwerveRequest(0, 0, -0.10 * speed), true); // drive backwards
    } else if (pitch < -2.3) {
      drive.drive(new SwerveSubsystem.SwerveRequest(0, 0, 0.10 * speed), true); // drive forward
    } else {
      drive.drive(new SwerveSubsystem.SwerveRequest(0, 0, 0), true);
      isFinished = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return isFinished;
  }
}
