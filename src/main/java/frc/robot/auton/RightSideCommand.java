package frc.robot.auton;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LightSensor;
import frc.robot.Limelight;
import frc.robot.commands.AutonLimelight;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.GetOnChargeStation;
import frc.robot.commands.ScoreAuton;
import frc.robot.commands.SetClawPneumatics;
import frc.robot.commands.SetClawPreset;
import frc.robot.commands.TurnAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class RightSideCommand extends SequentialCommandGroup {
    protected ClawPneumatics clawPneumatics;
    protected SwerveSubsystem drive;
    protected ArmSubsystem arm;
    protected Pigeon2 gyro;
    protected Limelight lime;
    protected LightSensor sensor;

    protected String color;

    public RightSideCommand(ArmSubsystem arm, ClawPneumatics clawPneumatics, SwerveSubsystem drive, Pigeon2 gyro, Limelight lime, LightSensor sensor){
        this.clawPneumatics = clawPneumatics;
        this.drive = drive;
        this.arm = arm;
        this.gyro = gyro;
        this.lime = lime;
        this.sensor = sensor;
    
        addCommands(
            new SetClawPreset(arm, 4),
            new SetClawPneumatics(clawPneumatics, 1, arm),
            new DriveDistance(drive, Constants.encoderToOutsideCommunityDistance, 0, 0.5, false).alongWith(
                new SetClawPreset(arm, 1)),
            // new SetClawPreset(arm, 1),
            new TurnAngle(drive, 6),
            new SetClawPneumatics(clawPneumatics, 1, arm),
            new AutonLimelight(drive, lime, arm, clawPneumatics, sensor),
            new SetClawPreset(arm, 1),
            //new DriveDistance(drive, 1.5, Math.PI / 2, 0.5, false),
            new TurnAngle(drive,  Math.PI)
            // new DriveDistance(drive, 60, Math.PI, 0.5, false),
            // new SetClawPreset(arm, 4),
            // new ScoreAuton(drive, lime, arm, clawPneumatics)
        );
    }
}

