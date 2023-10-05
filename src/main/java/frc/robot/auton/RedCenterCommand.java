package frc.robot.auton;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.commands.AutonTime;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.GetOnChargeStation;
import frc.robot.commands.SetClawPneumatics;
import frc.robot.commands.SetClawPreset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class RedCenterCommand extends SequentialCommandGroup {
  protected ClawPneumatics clawPneumatics;
  protected SwerveSubsystem drive;
  protected ArmSubsystem arm;
  protected Pigeon2 gyro;
  private SwerveControllerCommand swerve;
  private boolean brake;
  private SwerveOdometry odometry;
  private SwerveDriveKinematics kinematics;
  private ProfiledPIDController thetaController;

  public RedCenterCommand(
      ArmSubsystem arm,
      ClawPneumatics clawPneumatics,
      SwerveSubsystem drive,
      Pigeon2 gyro,
      SwerveOdometry odometry) {
    this.clawPneumatics = clawPneumatics;
    this.drive = drive;
    this.arm = arm;
    this.gyro = gyro;
    this.odometry = odometry;
    this.thetaController =
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, 3));
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.kinematics =
        new SwerveDriveKinematics(
            new Translation2d(Constants.Measurement.TRACKWIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(Constants.Measurement.TRACKWIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(-Constants.Measurement.TRACKWIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(-Constants.Measurement.TRACKWIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0));
    TrajectoryConfig config =
        new TrajectoryConfig(3, 3)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kinematics);

    // config.setReversed(true);

    addCommands(
        new SetClawPreset(arm, 4),
        new SetClawPneumatics(clawPneumatics, 1, arm),
        new DriveDistance(drive, Constants.Distance.TO_RED_CHARGE_STATION, 0, 0.4, true)
            .alongWith(new SetClawPreset(arm, 1)),
        new AutonTime(1.3),
        new DriveDistance(drive, 52, Math.PI, 0.19, true),
        new GetOnChargeStation(drive, gyro).repeatedly());
  }
}
