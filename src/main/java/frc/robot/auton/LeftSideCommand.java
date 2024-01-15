package frc.robot.auton;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LightSensor;
import frc.robot.Limelight;
import frc.robot.commands.AutonLimelight;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.SetClawPneumatics;
import frc.robot.commands.SetClawPreset;
import frc.robot.commands.TurnAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class LeftSideCommand extends SequentialCommandGroup {
  protected ClawPneumatics clawPneumatics;
  protected SwerveSubsystem drive;
  protected ArmSubsystem arm;
  protected Pigeon2 gyro;
  protected Limelight lime;
  protected LightSensor sensor;

  public LeftSideCommand(
      ArmSubsystem arm,
      ClawPneumatics clawPneumatics,
      SwerveSubsystem drive,
      Pigeon2 gyro,
      Limelight lime,
      LightSensor sensor) {
    this.clawPneumatics = clawPneumatics;
    this.drive = drive;
    this.arm = arm;
    this.gyro = gyro;
    this.lime = lime;
    this.sensor = sensor;
    addCommands(
        new SetClawPreset(arm, 4),
        new SetClawPneumatics(clawPneumatics, 1, arm),
        new DriveDistance(drive, Constants.Distance.TO_OUTSIDE_COMMUNITY, 0, 0.5, false)
            .alongWith(new SetClawPreset(arm, 1)),
        new TurnAngle(drive, 6),
        new SetClawPneumatics(clawPneumatics, 1, arm),
        new AutonLimelight(drive, lime, arm, clawPneumatics, sensor),
        new SetClawPreset(arm, 1),
        new TurnAngle(drive, Math.PI));
  }
}
