package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LightSensor;
import frc.robot.Limelight;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoreAuton extends Command {
  protected SwerveSubsystem drive;
  protected Limelight lime;
  protected ArmSubsystem arm;
  protected ClawPneumatics claw;
  protected LightSensor sensor;
  protected boolean isFinished;

  /*
   * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
   *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
   */
  public ScoreAuton(SwerveSubsystem drive, Limelight lime, ArmSubsystem arm, ClawPneumatics claw) {
    this.drive = drive;
    this.lime = lime;
    this.arm = arm;
    this.claw = claw;
  }

  //    initialize() - This method sets up the command and is called immediately before the command
  // is executed for the first time and every subsequent time it is started .
  //  Any initialization code should be here.
  public void initialize() {
    isFinished = false;
    this.lime.isTurningDone = false;
    this.lime.turnController.setP(0.001);
  }

  /*
   *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
   *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
   */
  public void execute() {
    if (this.lime.pickupTimer.get() >= 1) {
      claw.setHigh();
    } else {
      isFinished = lime.score(drive, arm, claw, true);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return isFinished;
  }
}
