package frc.robot.auton;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LightSensor;
import frc.robot.Limelight;
import frc.robot.commands.AutonLimelight;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.GetOnChargeStation;
import frc.robot.commands.SetClawPneumatics;
import frc.robot.commands.SetClawPreset;
import frc.robot.commands.TurnAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class SideCommand extends SequentialCommandGroup {
    protected ClawPneumatics clawPneumatics;
    protected SwerveSubsystem drive;
    protected ArmSubsystem arm;
    protected Pigeon2 gyro;
    protected Limelight lime;
    protected LightSensor sensor;

    public SideCommand(ArmSubsystem arm, ClawPneumatics clawPneumatics, SwerveSubsystem drive, Pigeon2 gyro, Limelight lime, LightSensor sensor){
        this.clawPneumatics = clawPneumatics;
        this.drive = drive;
        this.arm = arm;
        this.gyro = gyro;
        this.lime = lime;
        this.sensor = sensor;
        addCommands(
            new SetClawPreset(arm, 4),
            new SetClawPneumatics(clawPneumatics, 1),
            new DriveDistance(drive, Constants.encoderToOutsideCommunityDistance, 0).alongWith(
                new SetClawPreset(arm, 1))
            // ,new TurnAngle(drive, Math.PI),
            // new AutonLimelight(drive, lime, arm, clawPneumatics, sensor)
         
        );
    }
}

