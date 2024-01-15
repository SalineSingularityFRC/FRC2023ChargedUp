package frc.robot.auton;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends SequentialCommandGroup {
  protected ClawPneumatics clawPneumatics;
  protected SwerveSubsystem drive;
  protected ArmSubsystem arm;
  protected Pigeon2 gyro;
  private SwerveControllerCommand swerve;
  private boolean brake;
  private SwerveOdometry odometry;
  private SwerveDriveKinematics kinematics;
  private ProfiledPIDController thetaController;
  private Trajectory trajectory;
  public static TrajectoryConfig config;

  public SwerveCommand(
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
    this.trajectory = trajectory;
    this.thetaController =
        new ProfiledPIDController(0.025, 0, 0, new TrapezoidProfile.Constraints(Math.PI, 3));
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.kinematics =
        new SwerveDriveKinematics(
            new Translation2d(
                Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(
                Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(
                -Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0),
            new Translation2d(
                -Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0));
    this.config =
        new TrajectoryConfig(1, 1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kinematics);
    double[] swerve_xcontroller_gains = Constants.PidGains.SwerveCommand.X_CONTROLLER;
    double[] swerve_ycontroller_gains = Constants.PidGains.SwerveCommand.Y_CONTROLLER;
    addCommands(
        // new SwerveControllerCommand(
        //     trajectory,
        //     odometry::position,
        //     kinematics,
        //     new PIDController(swerve_xcontroller_gains[0], swerve_xcontroller_gains[1],
        // swerve_xcontroller_gains[2]),
        //     new PIDController(swerve_ycontroller_gains[0], swerve_ycontroller_gains[1],
        // swerve_ycontroller_gains[2]),
        //     thetaController,
        //     drive::setModuleStates,
        //     drive),
        new DriveDistance(drive, 2, 0, 0.4, true));
  }
}
