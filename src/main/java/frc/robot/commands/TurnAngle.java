package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.auton.AutonControlScheme;
import frc.robot.auton.RunAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnAngle extends CommandBase {

    protected SwerveSubsystem drive;
    private double angle;
    private boolean isFinished = false;
    private double finalAngle;
   /*
    * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
    *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
    */
    public TurnAngle(SwerveSubsystem drive, double angle) {
        this.drive = drive;
        this.angle = angle;
    }

    //    initialize() - This method sets up the command and is called immediately before the command is executed for the first time and every subsequent time it is started .
    //  Any initialization code should be here.
    public void initialize() {
        finalAngle = drive.getRobotAngle() + angle;
    }

    /*
     *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
     *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
     */
    public void execute() {
        if (drive.getRobotAngle() < finalAngle) { 
            drive.drive(new SwerveSubsystem.SwerveRequest(0.5, 0.0, 0.0));
        }
        else {
            isFinished = true;
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return isFinished;
    }
}