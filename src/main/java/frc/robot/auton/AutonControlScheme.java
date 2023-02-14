package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;


public class AutonControlScheme {

    protected ClawPneumatics clawPneumatics;
    protected SwerveSubsystem drive;
    protected String color;

    private double targetAngle;

    public AutonControlScheme(ClawPneumatics clawPneumatics, SwerveSubsystem drive, String color){
        this.clawPneumatics = clawPneumatics;
        this.drive = drive;
        this.color = color;

        // targetAngle = 0;
        // targetAngle = 180;
    }

    public void driveDistance(double distance) {
        double encoderValue = drive.getSwerveModule(0).getPosition();
        double targetEncoderValue = encoderValue + distance;
        while (encoderValue < targetEncoderValue) {
            drive.drive(new SwerveSubsystem.SwerveRequest(0, 0, 1), false);
            encoderValue = drive.getSwerveModule(0).getPosition();
        }
    }

        

    
}