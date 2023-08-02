package frc.robot.SwerveClasses;


import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveOdometry {
    SwerveDriveOdometry swerveOdometry;
    public Pigeon2 gyro;

    private final int FL = 0;
    private final int FR = 1;
    private final int BL = 2;
    private final int BR = 3;

    private final SwerveDriveKinematics swerveKinematics;

    private SwerveModule[] swerveModules = new SwerveModule[4];
    private final Translation2d[] vectorKinematics = new Translation2d[4];
    private SwerveSubsystem subsystem;
    public SwerveOdometry(SwerveSubsystem subsystem){
        this.subsystem = subsystem;
        gyro = subsystem.gyro;
        vectorKinematics[FL] = new Translation2d(Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0);
        vectorKinematics[FR] = new Translation2d(Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0);    
        vectorKinematics[BL] = new Translation2d(-Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0);   
        vectorKinematics[BR] = new Translation2d(-Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0);

        swerveKinematics = new SwerveDriveKinematics(vectorKinematics[FL], vectorKinematics[FR], vectorKinematics[BL], vectorKinematics[BR]);
        swerveModules[FL] = new SwerveModule(Constants.FL_Motor_ID, Constants.FL_ANGLE_ID, Constants.FL_CANCODER_ID, Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET, Constants.CANIVORE, Constants.FL_isInverted, "FL");
        swerveModules[FR] = new SwerveModule(Constants.FR_Motor_ID, Constants.FR_ANGLE_ID, Constants.FR_CANCODER_ID, Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET, Constants.CANIVORE, Constants.FR_isInverted, "FR");
        swerveModules[BL] = new SwerveModule(Constants.BL_Motor_ID, Constants.BL_ANGLE_ID, Constants.BL_CANCODER_ID, Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET, Constants.CANIVORE, Constants.BL_isInverted, "BR");
        swerveModules[BR] = new SwerveModule(Constants.BR_Motor_ID, Constants.BR_ANGLE_ID, Constants.BR_CANCODER_ID, Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET, Constants.CANIVORE, Constants.BR_isInverted, "BL");

        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, gyro.getRotation2d(),  new SwerveModulePosition[] {
            new SwerveModulePosition(swerveModules[FL].getEncoderPosition(), new Rotation2d(swerveModules[FL].getPosition())),
            new SwerveModulePosition(swerveModules[FR].getEncoderPosition(), new Rotation2d(swerveModules[FR].getPosition())),
            new SwerveModulePosition(swerveModules[BL].getEncoderPosition(), new Rotation2d(swerveModules[BL].getPosition())),
            new SwerveModulePosition(swerveModules[BR].getEncoderPosition(), new Rotation2d(swerveModules[BR].getPosition())),
          }, new Pose2d(0, 0, new Rotation2d()));;

    }

    public void update(){
        swerveOdometry.update(gyro.getRotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(swerveModules[FL].getPosition() , new Rotation2d(swerveModules[FL].getEncoderPosition())),
            new SwerveModulePosition(swerveModules[FR].getPosition(), new Rotation2d(swerveModules[FR].getEncoderPosition())),
            new SwerveModulePosition(swerveModules[BL].getPosition(), new Rotation2d(swerveModules[BL].getEncoderPosition())),
            new SwerveModulePosition(swerveModules[BR].getPosition(), new Rotation2d(swerveModules[BR].getEncoderPosition())),
          });
    }

    
    public Pose2d position(){
        return swerveOdometry.getPoseMeters();
    }
    public double getX(){
        return position().getX();
    }
    public double getY(){
        return position().getY();
    }
    
    public void resetPosition(){
        swerveOdometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(swerveModules[FL].getEncoderPosition(), new Rotation2d(swerveModules[FL].getPosition())),
            new SwerveModulePosition(swerveModules[FR].getEncoderPosition(), new Rotation2d(swerveModules[FR].getPosition())),
            new SwerveModulePosition(swerveModules[BL].getEncoderPosition(), new Rotation2d(swerveModules[BL].getPosition())),
            new SwerveModulePosition(swerveModules[BR].getEncoderPosition(), new Rotation2d(swerveModules[BR].getPosition())),
          }, new Pose2d(0, 0, new Rotation2d()));
    }
}
