package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.auton.AutonControlScheme;
import frc.robot.auton.RunAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveDistance extends CommandBase {

    protected SwerveSubsystem drive;
    private double distance;
    private double angle;
    private boolean isFinished = false;
   /*
    * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
    *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
    */
    public DriveDistance(SwerveSubsystem drive, double distance, double angle) {
        this.drive = drive;
        this.distance = distance;
        this.angle = angle;
    }

    //    initialize() - This method sets up the command and is called immediately before the command is executed for the first time and every subsequent time it is started .
    //  Any initialization code should be here.
    public void initialize() {
    }

    /*
     *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
     *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
     */
    public void execute() {
        double startingEncoderValue = drive.getSwerveModule(0).getPosition();
        double changeInEncoderValue = 0;

        double rotations;
        double startingAngle = drive.getRobotAngle();
        double x = -Math.sin(angle);
        double y = Math.cos(angle);

        if (changeInEncoderValue <= distance) {
            double difference = drive.getRobotAngle() - startingAngle; 
            if (difference > 0.01) { // robot is facing left of the desired angle
                rotations = 0.1;
            }
            else if (difference < 0.01) { // robot is facing right of the desired angle
                rotations = -0.1;
            }
            else { // robot is (relatively) straight
                rotations = 0;
            }

            drive.drive(new SwerveSubsystem.SwerveRequest(rotations, x, y), false);
            changeInEncoderValue = Math.abs(drive.getSwerveModule(0).getPosition() - startingEncoderValue);
        }
        else {
            drive.drive(new SwerveSubsystem.SwerveRequest(0, 0, 0), false);
            isFinished = true;
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}