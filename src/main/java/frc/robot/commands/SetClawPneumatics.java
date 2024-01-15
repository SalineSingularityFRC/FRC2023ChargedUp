package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;

public class SetClawPneumatics extends Command {

  protected ClawPneumatics clawPneumatics;
  private int position;
  private Timer timer;
  private ArmSubsystem arm;

  /*
   * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
   *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
   */

  public SetClawPneumatics(ClawPneumatics clawPneumatics, int position, ArmSubsystem arm) {
    this.position = position;
    this.clawPneumatics = clawPneumatics;
    this.arm = arm;
    timer = new Timer();
  }

  //    initialize() - This method sets up the command and is called immediately before the command
  // is executed for the first time and every subsequent time it is started .
  //  Any initialization code should be here.
  public void initialize() {
    if (position == 1) {
      arm.smallArmMotorPosition -= Constants.Speed.ARM * 9;
      arm.maintainPosition();
    }
    timer.reset(); // Reset the timer when the command starts
    timer.start(); // Start the timer when the command starts
  }

  /*
   *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
   *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
   */
  public void execute() {
    if (position == 1) { // open
      if (timer.get() >= 0.1) {
        clawPneumatics.setLow();
      }

    } else if (position == 2) {
      clawPneumatics.setHigh();
    } else if (position == 3) {
      clawPneumatics.setOff();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return timer.get() >= 0.2;
  }
}
