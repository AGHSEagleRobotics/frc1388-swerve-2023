// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class SwerveDriveTrain extends SubsystemBase {

  // private final int FR_INDEX = 0; // XXX Maybe add these latter in:    m_frontRight.setSwerveModuleStates(states[FR_INDEX]);
  // private final int FL_INDEX = 1;
  // private final int BL_INDEX = 2;
  // private final int BR_INDEX = 3;


  private final double ROBOT_WIDTH = 0;
  private final double ROBOT_LENGTH = 0;

  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

  private final Translation2d m_frontRightTranslation = new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);
  private final Translation2d m_frontLeftTranslation = new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  private final Translation2d m_backLeftTranslation = new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  private final Translation2d m_backRightTranslation = new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);

  private final Translation2d[] m_swerveTranslation2d = {
    m_frontRightTranslation,
    m_frontLeftTranslation,
    m_backLeftTranslation,
    m_backRightTranslation
  };

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2d);

  private final ADIS16470_IMU m_gyro;

  // private final SwerveDriveOdometry swerveOdometry;

  /** Creates a new SwerveDriveTrain. */
  public SwerveDriveTrain(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight, ADIS16470_IMU gyro) {
    m_frontRight = frontRight;
    m_frontLeft =  frontLeft;
    m_backLeft =   backLeft;
    m_backRight =  backRight;

    m_gyro = gyro;
    m_gyro.calibrate();
    m_gyro.reset();
    m_gyro.setYawAxis(IMUAxis.kX);

    // swerveOdometry = new SwerveDriveOdometry(new SwerveDriveKinematics(m_frontRightTranslation, m_frontLeftTranslation, m_backLeftTranslation, m_backRightTranslation), new Rotation2d(0), new SwerveModulePosition());
  }

  public void drive(double xVelocity, double yVelocity, double omega) {
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getGyroHeading());
    // ChassisSpeeds normalSpeeds = new ChassisSpeeds(xVelocity, yVelocity, omega); XXX -> for not field relitive <-
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);

    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);
  }

  private Rotation2d getGyroHeading() {
    return new Rotation2d(Math.toRadians(Math.IEEEremainder(m_gyro.getAngle(), 360)));
  }

  public void resetGyroHeading() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    System.out.println("current gyro angle: " + getGyroHeading().getDegrees()); //XXX temp removed
    // This method will be called once per scheduler run
  }
}
