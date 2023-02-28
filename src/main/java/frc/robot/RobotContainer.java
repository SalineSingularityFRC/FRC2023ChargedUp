// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auton.CenterCommand;
import frc.robot.auton.SideCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  SendableChooser<Boolean> isCenter = new SendableChooser<>();
  private CenterCommand centerCommand;
  private SideCommand sideCommand;
  protected ClawPneumatics clawPneumatics;
  protected SwerveSubsystem drive;
  protected ArmSubsystem arm;
  protected Pigeon2 gyro;

  public RobotContainer(ArmSubsystem arm, ClawPneumatics clawPneumatics, SwerveSubsystem drive, Pigeon2 gyro) {
    configureBindings();
    SmartDashboard.putData(isCenter);
    this.clawPneumatics = clawPneumatics;
    this.drive = drive;
    this.arm = arm;
    this.gyro = gyro;
    this.centerCommand = new CenterCommand(arm, clawPneumatics, drive, gyro);
    this.sideCommand = new SideCommand(arm, clawPneumatics, drive, gyro);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    if (isCenter.getSelected()) {
      return centerCommand;
    }
    else {
      return sideCommand;
    }
    // return Commands.print("No autonomous command configured");
  }
}