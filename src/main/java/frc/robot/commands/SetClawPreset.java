package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import edu.wpi.first.wpilibj.smartdashboard.*;


public class SetClawPreset extends CommandBase {

    protected ArmSubsystem arm;
    private int position;
    private Timer timer;

   /*
    * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
    *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
    */
    public SetClawPreset(ArmSubsystem arm, int position) {
        this.position = position;
        this.arm = arm;
        timer = new Timer();
    }

    //    initialize() - This method sets up the command and is called immediately before the command is executed for the first time and every subsequent time it is started .
    //  Any initialization code should be here.
    public void initialize() {
        timer.reset(); // Reset the timer when the command starts
        timer.start(); // Start the timer when the command starts
    }

    /*
     *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
     *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
     */
    public void execute() {

        if (position == 1) {
            arm.defaultTarget();
        }
        else if (position == 2) {
            arm.pickupTarget();
        }
        else if (position == 3) {
            arm.mediumTarget();
        }
        else if (position == 4) {
            arm.highTarget(timer);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return timer.get() >= 2.5;
    }
}