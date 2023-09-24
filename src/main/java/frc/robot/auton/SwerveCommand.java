package frc.robot.auton;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;

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
      SwerveOdometry odometry,
      Trajectory trajectory) {
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
            new Translation2d(Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0),
            new Translation2d(Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0),
            new Translation2d(-Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0),
            new Translation2d(-Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0));
        this.config =
        new TrajectoryConfig(1, 1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kinematics);

    //config.setReversed(true);
   
        // TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(Math.PI)),
        //     // Pass through these two interior waypoints, making an 's' curve path
        //     List.of(
        //         //new Translation2d(1, 0),
        //         //new Translation2d(1.5, 0),
        //         //new Translation2d(2, 0),
        //         //new Translation2d(2.5, 0),
        //         //new Translation2d(3.0, 0)
        //         // new Translation2d(-1, 0.2)
        //         ), // new Translation2d(1, 1), new Translation2d(2, -1)),
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(-3.5, 0, new Rotation2d(Math.PI)),
        //     config);

    addCommands(
        new SwerveControllerCommand(
            trajectory,
            odometry::position,
            kinematics,
            new PIDController(0.0001, 0, 0),
            new PIDController(0.0001, 0, 0),
            thetaController,
            drive::setModuleStates,
            drive),
        new DriveDistance(drive, 0, 0, 0.4, true));
  }
}