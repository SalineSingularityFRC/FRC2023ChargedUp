// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SwerveClasses.SwerveOdometry;
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
  private LightSensor cubelightSensor;
  private LightSensor conelightSensor;
  private SwerveOdometry odometry;

  // private CANdleSystem candle;

  @Override
  public void robotInit() {

    // Required to allow power to the switchable port on the power distrubution hub and allow sensor
    // to use max power
    PowerDistribution PD = new PowerDistribution();
    PD.setSwitchableChannel(true);

    robotSubsystem = new SwerveSubsystem();
    teleopDrive = new Gamepad(Constants.DRIVE_CONTROLLER, Constants.ARM_CONTROLLER);
    odometry = new SwerveOdometry(robotSubsystem);

    arm = new ArmSubsystem(false, true);
    clawPneumatics = new ClawPneumatics(9, 10, arm); // check these channel #s later

    limelight = new Limelight();
    cubelightSensor = new LightSensor(Constants.CUBE_SENSOR_CHANNEL);
    conelightSensor = new LightSensor(Constants.CONE_SENSOR_CHANNEL); 

    m_robotContainer =
        new RobotContainer(
            arm,
            clawPneumatics,
            robotSubsystem,
            robotSubsystem.gyro,
            limelight,
            cubelightSensor,
            conelightSensor,
            odometry);
    robotSubsystem.resetGyro();

    // candle = new CANdleSystem();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("ROBOT ANGLE ALL THE TIME", robotSubsystem.getRobotAngle());
    SmartDashboard.putNumber("RAW GYRO", robotSubsystem.gyro.getAngle() % 360);
    SmartDashboard.putNumber("ROBOT GYGROZERO ANGLE ALL THE TIME", robotSubsystem.gyroZero);
    odometry.update();
    SmartDashboard.putNumber("Odometry X", odometry.getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getY());
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
    // robotSubsystem.resetGyro();
    robotSubsystem.setBrakeMode();
    // odometry.resetPosition();
    if (m_autonomousCommand != null) {

      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // CommandScheduler.getInstance().setDefaultCommand( (Subsystem)
    //  m_robotContainer.getDrivetrainSubsystem(),
    // m_robotContainer.getDefaultCommand());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    robotSubsystem.setCoastMode();
  }

  @Override
  public void teleopPeriodic() {
    teleopDrive.swerveDrive(robotSubsystem, limelight, arm, clawPneumatics, cubelightSensor, conelightSensor);
    teleopDrive.arm(arm);
    teleopDrive.armPneumatics(clawPneumatics, cubelightSensor,conelightSensor, arm);
    //teleopDrive.swerveDrive(robotSubsystem, limelight, arm, clawPneumatics, cubelightSensor, conelightSensor);
    limelight.runLimelight();
    SmartDashboard.putBoolean("is target found", limelight.getIsTargetFound());
    SmartDashboard.putNumber("gyro", robotSubsystem.getRobotAngle());

    SmartDashboard.putBoolean("is sensed", cubelightSensor.isSensed());
    SmartDashboard.putNumber("volts", cubelightSensor.volts());

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
