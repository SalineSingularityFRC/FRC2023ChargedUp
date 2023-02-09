// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.SmallArm;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SwerveSubsystem robotSubsystem;
  private Gamepad teleopDrive;

  private BigArm bigArm;
  private SmallArm smallArm;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    robotSubsystem = new SwerveSubsystem();
    teleopDrive = new Gamepad(Constants.DRIVE_CONTROLLER, Constants.ARM_CONTROLLER);

    bigArm = new BigArm(Constants.BIG_ARM_Motor_ID, Constants.CANBUS, false, 0);
    smallArm = new SmallArm(Constants.SMALL_ARM_MOTOR_ID, Constants.CANBUS, false, 0) ;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {} 

  @Override
  public void teleopInit() {
    // CommandScheduler.getInstance().setDefaultCommand( (Subsystem) m_robotContainer.getDrivetrainSubsystem(), m_robotContainer.getDefaultCommand());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    teleopDrive.swerveDrive(robotSubsystem);
    
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
