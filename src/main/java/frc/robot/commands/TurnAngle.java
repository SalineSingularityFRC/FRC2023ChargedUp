package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnAngle extends CommandBase {

  protected SwerveSubsystem drive;
  private double angle;
  private boolean isFinished = false;
  private double finalAngle;
  private PIDController controller;

  /*
   * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
   *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
   */
  public TurnAngle(SwerveSubsystem drive, double angle) {
    this.drive = drive;
    this.angle = angle;
    this.controller = new PIDController(Math.PI / 6, 0, 0);
  
  }

  //    initialize() - This method sets up the command and is called immediately before the command
  // is executed for the first time and every subsequent time it is started .
  //  Any initialization code should be here.
  public void initialize() {
    this.controller.setSetpoint(angle);
  }

  /*
   *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
   *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
   */
  public void execute() {

    if (!this.controller.atSetpoint()) {
      drive.drive(
          new SwerveSubsystem.SwerveRequest(
              -this.controller.calculate(drive.getRobotAngle()), 0.0, 0.0),
          true);
    } else {
      isFinished = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return isFinished;
  }
}
