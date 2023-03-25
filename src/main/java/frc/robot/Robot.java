// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  private Limelight limelight;
  private LightSensor lightSensor; 
  //private CANdleSystem candle;

  @Override
  public void robotInit() {

    //Required to allow power to the switchable port on the power distrubution hub and allow sensor to use max power
    PowerDistribution PD = new PowerDistribution();
    PD.setSwitchableChannel(true); 


    robotSubsystem = new SwerveSubsystem();
    teleopDrive = new Gamepad(Constants.DRIVE_CONTROLLER, Constants.ARM_CONTROLLER);

    arm = new ArmSubsystem(false, true);
    clawPneumatics = new ClawPneumatics(9, 6); // check these channel #s later

    m_robotContainer = new RobotContainer(arm, clawPneumatics, robotSubsystem, robotSubsystem.gyro);
    robotSubsystem.resetGyro();

    limelight = new Limelight();
    lightSensor = new LightSensor();
    //candle = new CANdleSystem();
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
     
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    robotSubsystem.resetGyro();
    
    if (m_autonomousCommand != null) {
      
      m_autonomousCommand.schedule();
    }
        
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    //CommandScheduler.getInstance().setDefaultCommand( (Subsystem)
    // m_robotContainer.getDrivetrainSubsystem(),
    // m_robotContainer.getDefaultCommand());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    teleopDrive.swerveDrive(robotSubsystem, limelight, arm, clawPneumatics, lightSensor);
    teleopDrive.arm(arm);
    teleopDrive.armPneumatics(clawPneumatics, lightSensor);

    limelight.runLimelight();
    SmartDashboard.putBoolean("is target found", limelight.getIsTargetFound());
    SmartDashboard.putNumber("gyro", robotSubsystem.getRobotAngle());

    SmartDashboard.putBoolean("is sensed", lightSensor.isSensed());
    SmartDashboard.putNumber("volts", lightSensor.volts());

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
