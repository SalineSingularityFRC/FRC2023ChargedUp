package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;

public class SwerveDistance extends CommandBase {

  protected SwerveSubsystem drive;
  private double distance;
  private double angle;
  private boolean isFinished = false;
  private double startingEncoderValue;
  private double changeInEncoderValue = 0;
  private double speed;
  private PIDController controller;
  private SwerveControllerCommand swerve;
  private boolean brake;
  private SwerveOdometry odometry;
  private SwerveDriveKinematics kinematics;
  private ProfiledPIDController thetaController;

  /*
   * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
   *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
   */
  public SwerveDistance(
      SwerveSubsystem drive, double distance, double angle, double speed, boolean brake) {
    this.drive = drive;
    this.distance = distance;
    this.angle = angle;
    this.speed = speed;
    this.brake = brake;
    this.controller = new PIDController(0.1, 0, 0);
    this.odometry = new SwerveOdometry(drive);
  }

  //    initialize() - This method sets up the command and is called immediately before the command
  // is executed for the first time and every subsequent time it is started .
  //  Any initialization code should be here.
  public void initialize() {
    this.thetaController =
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, 3));
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    startingEncoderValue = drive.getSwerveModule(0).getPosition();
    changeInEncoderValue = 0;
    this.controller.setSetpoint(distance);
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

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(), // new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 10, new Rotation2d(0)),
            config);
    this.swerve =
        new SwerveControllerCommand(
            trajectory,
            odometry::position,
            kinematics,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            thetaController,
            drive::setModuleStates,
            drive);
    if (this.brake) {
      drive.setBrakeMode();
    } else {
      drive.setCoastMode();
    }
  }

  /*
   *   execute() - This method is called periodically (about every 20ms) and does the work of the command. Sometimes, if there is a position a
   *  subsystem is moving to, the command might set the target position for the subsystem in initialize() and have an empty execute() method.
   */
  public void execute() {
    // SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    // desiredStates[0] = new SwerveModuleState(0.5, new Rotation2d(0));
    // desiredStates[1] = new SwerveModuleState(.5, new Rotation2d(0));
    // desiredStates[2] = new SwerveModuleState(.5, new Rotation2d(0));
    // desiredStates[3] = new SwerveModuleState(.5, new Rotation2d(0));
    // this.drive.setModuleStates(
    //     desiredStates
    // );
    this.swerve.execute();
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return isFinished;
  }
}
