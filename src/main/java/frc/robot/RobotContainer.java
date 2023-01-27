// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.input.XboxController;

public class RobotContainer {
  private final SwerveSubsystem drivetrainSubsystem = new SwerveSubsystem();

  private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

  public RobotContainer() {
    primaryController.getLeftXAxis().setInverted(true);
    primaryController.getRightXAxis().setInverted(true);

    // CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, defaultDrive);
    

    // configureButtonBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  // private void configureButtonBindings() {
  //   primaryController.getBackButton().whenPressed(
  //           () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
  //   );
  //   primaryController.getStartButton().whenPressed(
  //           drivetrainSubsystem::resetWheelAngles
  //   );
  // }

  // private Axis getDriveForwardAxis() {
  //   return primaryController.getLeftYAxis();
  // }

  // private Axis getDriveStrafeAxis() {
  //   return primaryController.getLeftXAxis();
  // }

  // private Axis getDriveRotationAxis() {
  //   return primaryController.getRightXAxis();
  // }

  // public DrivetrainSubsystem getDrivetrainSubsystem() {
  //   return drivetrainSubsystem;
  // }

  public XboxController getPrimaryController() {
    return primaryController;
  }
}
