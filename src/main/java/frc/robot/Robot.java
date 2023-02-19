// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.auton.RunAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private SwerveSubsystem robotSubsystem;

  private Gamepad teleopDrive;

  private ArmSubsystem arm;
  private ClawPneumatics clawPneumatics;
  private RunAuton runAuton;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    robotSubsystem = new SwerveSubsystem();

    teleopDrive = new Gamepad(Constants.DRIVE_CONTROLLER, Constants.ARM_CONTROLLER);

    arm = new ArmSubsystem(false, false);
    clawPneumatics = new ClawPneumatics(9, 6); // check these channel #s later

    //runAuton = new RunAuton(clawPneumatics, robotSubsystem, "blue"); // CHANGE COLOR LATER
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    /* 
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //may not be needed
    //robotSubsystem.resetGyro(); //Only for testing, need to make sure that gyro value is set to 0 on init
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      SmartDashboard.putNumber("Auton Turned On", 1);
    }
    
    runAuton.TestAutonCommands();
    */
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    // CommandScheduler.getInstance().setDefaultCommand( (Subsystem)
    // m_robotContainer.getDrivetrainSubsystem(),
    // m_robotContainer.getDefaultCommand());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      SmartDashboard.putBoolean("Auton Turned Off", true); //May not be needed: Checking to see auton is turned off when switch to teleop
    }
  
  }

  @Override
  public void teleopPeriodic() {
    teleopDrive.swerveDrive(robotSubsystem);
    teleopDrive.arm(arm);
    teleopDrive.armPneumatics(clawPneumatics);

    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
