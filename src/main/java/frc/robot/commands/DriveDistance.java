package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveDistance extends CommandBase {

  protected SwerveSubsystem drive;
  private double distance;
  private double angle;
  private boolean isFinished = false;
  private double startingEncoderValue;
  private double changeInEncoderValue = 0;
  private double speed;
  private PIDController controller;

  private boolean brake;

  /*
   * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
   *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
   */
  public DriveDistance(
      SwerveSubsystem drive, double distance, double angle, double speed, boolean brake) {
    this.drive = drive;
    this.distance = distance;
    this.angle = angle;
    this.speed = speed;
    this.brake = brake;
    this.controller = new PIDController(0.1, 0, 0);
  }

  //    initialize() - This method sets up the command and is called immediately before the command
  // is executed for the first time and every subsequent time it is started .
  //  Any initialization code should be here.
  public void initialize() {
    startingEncoderValue = drive.getSwerveModule(0).getPosition();
    changeInEncoderValue = 0;
    this.controller.setSetpoint(distance);
    if (this.brake) {
      drive.setBrakeMode();
    } else {
      drive.setCoastMode();
    }
  }

  /*
   *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
   *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
   */
  public void execute() {

    double x = -Math.sin(angle);
    double y = Math.cos(angle);

    if (Math.abs(changeInEncoderValue) <= distance) {

      changeInEncoderValue = drive.getSwerveModule(0).getPosition() - startingEncoderValue;
      double multiplier = this.controller.calculate(changeInEncoderValue);

      x *= multiplier;
      y *= multiplier;
      if (x > speed) x = speed;
      if (y > speed) y = speed;
      if (x < -speed) x = -speed;
      if (y < -speed) y = -speed;
      drive.drive(new SwerveSubsystem.SwerveRequest(0, x, y), true);
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
