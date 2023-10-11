package frc.robot.auton;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LightSensor;
import frc.robot.Limelight;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.commands.SetClawPneumatics;
import frc.robot.commands.SetClawPreset;
import frc.robot.commands.TurnAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;

public class RightSideCommand extends SequentialCommandGroup {
  protected ClawPneumatics clawPneumatics;
  protected SwerveSubsystem drive;
  protected ArmSubsystem arm;
  protected Pigeon2 gyro;
  protected Limelight lime;
  protected LightSensor sensor;

  protected String color;

  public RightSideCommand(
      ArmSubsystem arm,
      ClawPneumatics clawPneumatics,
      SwerveSubsystem drive,
      Pigeon2 gyro,
      Limelight lime,
      LightSensor sensor,
      SwerveOdometry odometry) {
    this.clawPneumatics = clawPneumatics;
    this.drive = drive;
    this.arm = arm;
    this.gyro = gyro;
    this.lime = lime;
    this.sensor = sensor;
    SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(
                Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(
                Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(
                -Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(
                -Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0));
    TrajectoryConfig config =
        new TrajectoryConfig(1, 1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kinematics);
    Trajectory trajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(-.5, 0)), // new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-1, 0, new Rotation2d(Math.PI)),
            config);

    Trajectory trajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(-.5, 0)), // new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-2, 0, new Rotation2d(Math.PI)),
            config);
    addCommands(
        new SetClawPreset(arm, 4),
        new SetClawPneumatics(clawPneumatics, 1, arm),
        // new DriveDistance(drive, Constants.encoderToOutsideCommunityDistance, 0, 0.5, false)
        new SwerveCommand(arm, clawPneumatics, drive, gyro, odometry, trajectory1)
            .alongWith(new SetClawPreset(arm, 1)),
        new TurnAngle(drive, 0),
        new SwerveCommand(arm, clawPneumatics, drive, gyro, odometry, trajectory2),
        new SetClawPreset(arm, 1)
        // //new TurnAngle(drive, 6),
        // new SetClawPneumatics(clawPneumatics, 1, arm),
        // new AutonLimelight(drive, lime, arm, clawPneumatics, sensor)
        // new SetClawPreset(arm, 1),
        // // new DriveDistance(drive, 1.5, Math.PI / 2, 0.5, false),
        // new TurnAngle(drive, Math.PI)

        // new DriveDistance(drive, 60, Math.PI, 0.5, false),
        // new SetClawPreset(arm, 4),
        // new ScoreAuton(drive, lime, arm, clawPneumatics)
        );
  }
}
