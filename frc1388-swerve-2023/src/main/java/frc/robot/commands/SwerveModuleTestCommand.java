// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModuleTestSubsystem;

public class SwerveModuleTestCommand extends CommandBase {

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_rightX;

  private final SwerveModuleTestSubsystem m_swerveModuleTest;

  /** Creates a new SwerveModuleTest. */
  public SwerveModuleTestCommand(Supplier<Double> leftY, Supplier<Double> rightX, SwerveModuleTestSubsystem swerveModuleTest) {
    m_leftY = leftY;
    m_rightX = rightX;

    m_swerveModuleTest = swerveModuleTest;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveModuleTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveModuleTest.driveMotor(m_leftY.get());
    m_swerveModuleTest.rotateMotor(m_rightX.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveModuleTest.driveMotor(0);
    m_swerveModuleTest.rotateMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
