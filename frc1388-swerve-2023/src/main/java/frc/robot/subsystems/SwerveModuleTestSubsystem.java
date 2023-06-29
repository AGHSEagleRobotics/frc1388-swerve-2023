// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleTestSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_rotationMotor;

  /** Creates a new SwerveModuleTest. */
  public SwerveModuleTestSubsystem(WPI_TalonFX driveMotor, WPI_TalonFX rotationMotor) {
    m_driveMotor = driveMotor;
    m_rotationMotor = rotationMotor;
  }

  public void driveMotor(double speed) {
    m_driveMotor.set(speed);
  }

  public void rotateMotor(double speed) {
    m_rotationMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
