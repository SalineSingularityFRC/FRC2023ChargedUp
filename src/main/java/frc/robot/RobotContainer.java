// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.auton.CenterCommand;
import frc.robot.auton.LeftSideCommand;
import frc.robot.auton.RightSideCommand;
import frc.robot.auton.SwerveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // SendableChooser<Boolean> isCenter = new SendableChooser<>();

  private CenterCommand centerCommand;
  private LeftSideCommand leftSideCommand;
  private RightSideCommand rightSideCommand;
  private SwerveCommand swerveCommand; 
  protected ClawPneumatics clawPneumatics;
  protected SwerveSubsystem drive;
  protected ArmSubsystem arm;
  protected Pigeon2 gyro;
  protected Limelight lime;
  protected LightSensor cubeSensor;
  protected LightSensor coneSensor;

  public RobotContainer(
      ArmSubsystem arm,
      ClawPneumatics clawPneumatics,
      SwerveSubsystem drive,
      Pigeon2 gyro,
      Limelight lime,
      LightSensor cubeSensor,
      LightSensor coneSensor,
      SwerveOdometry odometry) {
    configureBindings();
    // SmartDashboard.putData(isCenter);
    this.clawPneumatics = clawPneumatics;
    this.drive = drive;
    this.arm = arm;
    this.gyro = gyro;
    this.lime = lime;
    this.cubeSensor = cubeSensor;

    this.centerCommand = new CenterCommand(arm, clawPneumatics, drive, gyro, odometry);
    //this.swerveCommand = new SwerveCommand(arm, clawPneumatics, drive, gyro, odometry);
    this.leftSideCommand = new LeftSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor);
    this.rightSideCommand = new RightSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor, odometry);
    // isCenter.setDefaultOption("Center Auto", centerCommand);
    // isCenter.addOption("Side Auto", sideCommand);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    //return centerCommand;
    //return swerveCommand;
    return rightSideCommand;
    // return leftSideCommand;
  }
}
