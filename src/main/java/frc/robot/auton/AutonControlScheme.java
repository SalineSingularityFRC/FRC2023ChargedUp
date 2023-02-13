package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

import java.sql.Time;
import java.time.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;


public class AutonControlScheme {

    protected static ClawPneumatics clawPneumatics;
    protected static SwerveSubsystem drive;
    protected static String color;

    public AutonControlScheme(ClawPneumatics clawPneumatics, SwerveSubsystem drive, String color){
        this.clawPneumatics = clawPneumatics;
        this.drive = drive;
        this.color = color;
    }

        

    
}