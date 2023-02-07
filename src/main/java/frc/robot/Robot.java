// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SwerveSubsystem robotSubsystem;
  private Arm bigArm;
  private Arm smallArm;

  // private SwerveModule robotModule;
  private Joystick joystick;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // updateManager = new UpdateManager(m_robotContainer.getDrivetrainSubsystem());
    robotSubsystem = new SwerveSubsystem();
    joystick = new Joystick(0);

    bigArm = new Arm(Constants.BIG_ARM_Motor_ID, Constants.CANBUS, false, Constants.BIG_ARM_GEAR_RATIO);
    smallArm = new Arm(Constants.SMALL_ARM_MOTOR_ID, Constants.CANBUS, false, Constants.SMALL_ARM_GEAR_RATIO);
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
    robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(
      joystick.getRawAxis(Constants.rightJoystickXAxis), 
      -joystick.getRawAxis(Constants.leftJoystickXAxis), 
      -joystick.getRawAxis(Constants.leftJoystickYAxis)));

  
    if (joystick.getRawButtonPressed(1)) { //only sets the speed for one millisecond when you hold the button down
      bigArm.setSpeed(1/Constants.SPEED_DIVISOR);
    }
    else if (joystick.getRawButtonPressed(2)) {
      bigArm.setSpeed(-1/Constants.SPEED_DIVISOR);
    }
    else {
      bigArm.setSpeed(0);
    }

    

    SmartDashboard.putNumber("pov", joystick.getPOV());
    
    
    if (joystick.getPOV() == 0) {
      smallArm.setSpeed(1/Constants.SPEED_DIVISOR);
    }
    else if (joystick.getPOV() == 180) {
      smallArm.setSpeed(-1/Constants.SPEED_DIVISOR);
    }
    else { 
      smallArm.stop();
    }

    

    // targetAngle += joystick.getRawAxis(Constants.rightJoystickXAxis)/100;
    // targetAngle %= Math.PI * 2;
    // robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(
    //   (robotSubsystem.getRobotAngle()-targetAngle)/10, 
    //   -joystick.getRawAxis(Constants.leftJoystickXAxis), 
    //   -joystick.getRawAxis(Constants.leftJoystickYAxis)
    // ));
    
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
