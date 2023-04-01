package frc.robot.auton;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutonTime;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.GetOnChargeStation;
import frc.robot.commands.SetClawPneumatics;
import frc.robot.commands.SetClawPreset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class CenterCommand extends SequentialCommandGroup {
    protected ClawPneumatics clawPneumatics;
    protected SwerveSubsystem drive;
    protected ArmSubsystem arm;
    protected Pigeon2 gyro;

    public CenterCommand(ArmSubsystem arm, ClawPneumatics clawPneumatics, SwerveSubsystem drive, Pigeon2 gyro){
        this.clawPneumatics = clawPneumatics;
        this.drive = drive;
        this.arm = arm;
        this.gyro = gyro;

        addCommands(
           new SetClawPreset(arm, 4), 
            new SetClawPneumatics(clawPneumatics, 1, arm),
            new DriveDistance(drive, Constants.encoderToChargeDistance, 0, 0.4, true).alongWith(
                new SetClawPreset(arm, 1)),
            new AutonTime(1),
        
            new DriveDistance(drive, 51, Math.PI, 0.19, true),
        
            new GetOnChargeStation(drive, gyro).repeatedly()
        );
    }
}

