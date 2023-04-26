package frc.robot.SwerveClasses;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

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

    public SwerveOdometry(){
        gyro = new Pigeon2(Constants.GYRO_CANCODER_ID, Constants.CANIVORE);
        vectorKinematics[FL] = new Translation2d(Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0);
        vectorKinematics[FR] = new Translation2d(Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0);    
        vectorKinematics[BL] = new Translation2d(-Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0);   
        vectorKinematics[BR] = new Translation2d(-Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0);

        swerveKinematics = new SwerveDriveKinematics(vectorKinematics[FL], vectorKinematics[FR], vectorKinematics[BL], vectorKinematics[BR]);
        swerveModules[FL] = new SwerveModule(Constants.FL_Motor_ID, Constants.FL_ANGLE_ID, Constants.FL_CANCODER_ID, Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET, Constants.CANIVORE, Constants.FL_isInverted);
        swerveModules[FR] = new SwerveModule(Constants.FR_Motor_ID, Constants.FR_ANGLE_ID, Constants.FR_CANCODER_ID, Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET, Constants.CANIVORE, Constants.FR_isInverted);
        swerveModules[BL] = new SwerveModule(Constants.BL_Motor_ID, Constants.BL_ANGLE_ID, Constants.BL_CANCODER_ID, Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET, Constants.CANIVORE, Constants.BL_isInverted);
        swerveModules[BR] = new SwerveModule(Constants.BR_Motor_ID, Constants.BR_ANGLE_ID, Constants.BR_CANCODER_ID, Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET, Constants.CANIVORE, Constants.BR_isInverted);

        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, new Rotation2d(gyro.getYaw()),  new SwerveModulePosition[] {
            new SwerveModulePosition(swerveModules[FL].getEncoderPosition(), new Rotation2d(swerveModules[FL].getPosition())),
            new SwerveModulePosition(swerveModules[FR].getEncoderPosition(), new Rotation2d(swerveModules[FR].getPosition())),
            new SwerveModulePosition(swerveModules[BL].getEncoderPosition(), new Rotation2d(swerveModules[BL].getPosition())),
            new SwerveModulePosition(swerveModules[BR].getEncoderPosition(), new Rotation2d(swerveModules[BR].getPosition())),
          }, new Pose2d(0, 0, new Rotation2d()));;

    }

    public void update(){
        swerveOdometry.update(new Rotation2d(gyro.getYaw()), new SwerveModulePosition[] {
            new SwerveModulePosition(swerveModules[FL].getEncoderPosition(), new Rotation2d(swerveModules[FL].getPosition())),
            new SwerveModulePosition(swerveModules[FR].getEncoderPosition(), new Rotation2d(swerveModules[FR].getPosition())),
            new SwerveModulePosition(swerveModules[BL].getEncoderPosition(), new Rotation2d(swerveModules[BL].getPosition())),
            new SwerveModulePosition(swerveModules[BR].getEncoderPosition(), new Rotation2d(swerveModules[BR].getPosition())),
          });
    }

    public void resetPosition(){
        swerveOdometry.resetPosition(new Rotation2d(gyro.getYaw()), new SwerveModulePosition[] {
            new SwerveModulePosition(swerveModules[FL].getEncoderPosition(), new Rotation2d(swerveModules[FL].getPosition())),
            new SwerveModulePosition(swerveModules[FR].getEncoderPosition(), new Rotation2d(swerveModules[FR].getPosition())),
            new SwerveModulePosition(swerveModules[BL].getEncoderPosition(), new Rotation2d(swerveModules[BL].getPosition())),
            new SwerveModulePosition(swerveModules[BR].getEncoderPosition(), new Rotation2d(swerveModules[BR].getPosition())),
          }, new Pose2d(0, 0, new Rotation2d()));
    }
}
