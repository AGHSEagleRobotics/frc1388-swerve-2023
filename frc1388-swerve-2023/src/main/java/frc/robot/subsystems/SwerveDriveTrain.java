// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class SwerveDriveTrain extends SubsystemBase {

  // private final int FR_INDEX = 0; // XXX Maybe add these latter in:
  // m_frontRight.setSwerveModuleStates(states[FR_INDEX]);
  // private final int FL_INDEX = 1;
  // private final int BL_INDEX = 2;
  // private final int BR_INDEX = 3;

  private boolean hasGyroBeenReset = false;
  private Rotation2d latsRotation2d = new Rotation2d();

  private final double ROBOT_WIDTH = 0.4318; // in meters
  private final double ROBOT_LENGTH = 0.4318; // in meters

  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

  private final Translation2d m_frontRightTranslation = new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);
  private final Translation2d m_frontLeftTranslation = new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  private final Translation2d m_backLeftTranslation = new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  private final Translation2d m_backRightTranslation = new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);

  // private final Translation2d m_frontRightTranslation = new
  // Translation2d(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2);
  // private final Translation2d m_frontLeftTranslation = new
  // Translation2d(-ROBOT_WIDTH / 2, ROBOT_LENGTH / 2);
  // private final Translation2d m_backLeftTranslation = new
  // Translation2d(-ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2);
  // private final Translation2d m_backRightTranslation = new
  // Translation2d(ROBOT_WIDTH / 2, -ROBOT_WIDTH / 2);

  private final Translation2d[] m_swerveTranslation2d = {
      m_frontRightTranslation,
      m_frontLeftTranslation,
      m_backLeftTranslation,
      m_backRightTranslation
  };

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2d);
  private SwerveDriveOdometry m_odometry;

  // private final ADIS16470_IMU m_gyro;
  private final AHRS m_navxGyro;

  // private final SwerveDriveOdometry swerveOdometry;

  /** Creates a new SwerveDriveTrain. */
  public SwerveDriveTrain(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft,
      SwerveModule backRight, AHRS gyro) {
    m_frontRight = frontRight;
    m_frontLeft = frontLeft;
    m_backLeft = backLeft;
    m_backRight = backRight;

    m_navxGyro = gyro;
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        m_navxGyro.reset();
        m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            getGyroHeading(),
            new SwerveModulePosition[] {
                m_frontRight.getPosition(),
                m_frontLeft.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            new Pose2d(0, 0, new Rotation2d()));
      } catch (Exception e) {

      }
    }).start();

    // swerveOdometry = new SwerveDriveOdometry(new
    // SwerveDriveKinematics(m_frontRightTranslation, m_frontLeftTranslation,
    // m_backLeftTranslation, m_backRightTranslation), new Rotation2d(0), new
    // SwerveModulePosition());
  }

  public void drive(double xVelocity, double yVelocity, double omega) {
    SmartDashboard.putString("driver input", "x: " + xVelocity + "\t  y: " + yVelocity + "\t  omega: " + omega);
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getGyroHeading());
    // ChassisSpeeds normalSpeeds = new ChassisSpeeds(xVelocity, yVelocity, omega);
    // //XXX -> for not field relitive <-
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelativeSpeeds);
    // SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(normalSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);

    // SwerveModuleState testState = new SwerveModuleState(0, new
    // Rotation2d((Math.toRadians(45))));
    // m_frontRight.setSwerveModuleStates(testState);

    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);

    // smart dash board
    SmartDashboard.putNumber("x velocity", xVelocity);
    SmartDashboard.putNumber("y velocity", yVelocity);
    SmartDashboard.putNumber("omega", omega);

    // odo stuff
    m_odometry.update(
        getGyroHeading(),
        new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

    Pose2d currentrobpos = m_odometry.getPoseMeters();
    SmartDashboard.putNumber("odo robo x pos", currentrobpos.getX());
    SmartDashboard.putNumber("odo robo y pos", currentrobpos.getY());

  }

  private Rotation2d getGyroHeading() {
    if (!m_navxGyro.isCalibrating()) {
      latsRotation2d = new Rotation2d(Math.toRadians(Math.IEEEremainder(-m_navxGyro.getAngle(), 360)));
      return latsRotation2d;
    } else {
      return latsRotation2d;
    }
  }

  public void resetGyroHeading() {
    m_navxGyro.reset();
    hasGyroBeenReset = true;
  }

  public void resetOdometry() {
    m_odometry.resetPosition(getGyroHeading(),
        new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition() },
        new Pose2d());
  }

  @Override
  public void periodic() {
    // System.out.println("current gyro angle: " + getGyroHeading().getDegrees());
    // //XXX temp removed
    SmartDashboard.putNumber("current gyro angle", getGyroHeading().getDegrees());
    m_frontRight.periodic();
    m_frontLeft.periodic();
    m_backLeft.periodic();
    m_backRight.periodic();

    SmartDashboard.putNumber("XXXgyro", m_navxGyro.getAngle());

    SmartDashboard.putBoolean("has gyro been reset", hasGyroBeenReset);

    // if (Math.random() > 0.8) {
    // System.out.println(m_navxGyro.getFusedHeading());
    // }

    // XXX bad (?)
    // System.out.println("gyro heading:\t" + m_navxGyro.getFusedHeading());
    // System.out.println("gyro heading:\t" + m_navxGyro.getYaw());

    // This method will be called once per scheduler run
  }
}
