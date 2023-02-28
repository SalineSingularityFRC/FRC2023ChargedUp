package frc.robot.auton;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    public SideCommand(ArmSubsystem arm, ClawPneumatics clawPneumatics, SwerveSubsystem drive, Pigeon2 gyro){
        this.clawPneumatics = clawPneumatics;
        this.drive = drive;
        this.arm = arm;
        this.gyro = gyro;
    
        addCommands(new SetClawPreset(arm, 4));
        addCommands(new SetClawPneumatics(clawPneumatics, 1));
        addCommands(new DriveDistance(drive, 100, Math.PI)); // get out of community
        addCommands(new SetClawPreset(arm, 1));
        addCommands(new TurnAngle(drive, Math.PI));
    }
}

