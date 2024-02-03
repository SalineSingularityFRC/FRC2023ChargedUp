// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.auton.BlueCenterCommand;
import frc.robot.auton.LeftSideCommand;
import frc.robot.auton.RedCenterCommand;
import frc.robot.auton.RightSideCommand;
import frc.robot.auton.SwerveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.DriveController;

public class RobotContainer {

  private BlueCenterCommand blueCenterCommand;
  private RedCenterCommand redCenterCommand;
  private LeftSideCommand leftSideCommand;
  private RightSideCommand rightSideCommand;
  private SwerveCommand swerveCommand;
  private WaitCommand highTargetWait;
  private WaitCommand defaultTargetWait;
  protected ClawPneumatics clawPneumatics;
  protected SwerveSubsystem drive;
  protected ArmSubsystem arm;
  protected Pigeon2 gyro;
  protected Limelight lime;
  protected LightSensor cubeSensor;
  protected LightSensor coneSensor;
  private SendableChooser<Command> autonChooser;

  private CommandXboxController armController, driveController;

  public RobotContainer(
      ArmSubsystem arm,
      ClawPneumatics clawPneumatics,
      SwerveSubsystem drive,
      Pigeon2 gyro,
      Limelight lime,
      LightSensor cubeSensor,
      LightSensor coneSensor,
      SwerveOdometry odometry) {

    this.clawPneumatics = clawPneumatics;
    this.drive = drive;
    this.arm = arm;
    this.gyro = gyro;
    this.lime = lime;
    this.cubeSensor = cubeSensor;
  
    this.armController = new CommandXboxController(0);
    this.driveController = new CommandXboxController(1);
    this.highTargetWait = new WaitCommand(0.75);
    this.defaultTargetWait = new WaitCommand(0.5);

    this.blueCenterCommand = new BlueCenterCommand(arm, clawPneumatics, drive, gyro, odometry);
    this.redCenterCommand = new RedCenterCommand(arm, clawPneumatics, drive, gyro, odometry);

    this.swerveCommand = new SwerveCommand(arm, clawPneumatics, drive, gyro, odometry);
    this.leftSideCommand = new LeftSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor);
    this.rightSideCommand =
        new RightSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor, odometry);

    this.autonChooser = new SendableChooser<Command>();
    this.autonChooser.setDefaultOption("BlueCenter", blueCenterCommand);
    this.autonChooser.addOption("RedCenter", redCenterCommand);
    this.autonChooser.addOption("LeftSide", leftSideCommand);
    this.autonChooser.addOption("RightSide", rightSideCommand);
    this.autonChooser.addOption("SwerveDriveCommand", swerveCommand);

    SmartDashboard.putData("Auton Choices", autonChooser);
    configureBindings();
  }

  private void configureBindings() {
    armController.a().onTrue(arm.pickupTarget()
      .andThen(new WaitCommand(.5)).andThen(arm.setDebounceFalse()));
    armController.b().onTrue(arm.mediumTarget()
      .andThen(new WaitCommand(.5)).andThen(arm.setDebounceFalse()));
    armController.x().onTrue(arm.defaultTarget1().andThen(defaultTargetWait).andThen(arm.defaultTarget2())
      .andThen(new WaitCommand(.5)).andThen(arm.setDebounceFalse()));
    armController.y().onTrue(arm.highTarget1().andThen(highTargetWait).andThen(arm.highTarget2())
      .andThen(new WaitCommand(.75).andThen(arm.setDebounceFalse())));
    driveController.x().onTrue(drive.resetGyroCommand());
    drive.setDefaultCommand(new DriveController(drive, driveController::getRightX, driveController::getLeftX, driveController::getLeftY));
  }

  // public Command getAutonomousCommand() {
  //   return autonChooser.getSelected();
  // }

  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("Left");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPathWithEvents(path);
  }
}
