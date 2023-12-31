// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveTrain;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDriveTrain m_driveTrain;

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_leftX;
  private final Supplier<Double> m_rightX;

  // private final SlewRateLimiter m_translationalAccLimiter;
  // private final SlewRateLimiter m_rotationalAccLimiter;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(SwerveDriveTrain driveTrain, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX) {
    m_driveTrain = driveTrain;

    m_leftY = leftY;
    m_leftX = leftX;
    m_rightX = rightX;

    // XXX re-add these latter for acceleration limit
    // m_translationalAccLimiter = new SlewRateLimiter(2.0);
    // m_rotationalAccLimiter = new SlewRateLimiter(2.0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = -3.0 * MathUtil.applyDeadband(m_leftY.get(), 0.2);
    double yVelocity = -3.0 * MathUtil.applyDeadband(m_leftX.get(), 0.2); 
    double omega     = Math.PI * -MathUtil.applyDeadband(m_rightX.get(), 0.2);

    m_driveTrain.drive(xVelocity, yVelocity, omega); // max speed: 3 m/s transitional, pi rad/s (0.5 rotation/s) rotational (for now)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
