// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import javax.swing.text.html.HTMLDocument.RunElement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveTrain;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDriveTrain m_driveTrain;

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_leftX;
  private final Supplier<Double> m_rightX;
  private final Supplier<Double> m_rightTrig;


  // private final SlewRateLimiter m_translationalAccLimiter;
  // private final SlewRateLimiter m_rotationalAccLimiter;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(SwerveDriveTrain driveTrain, Supplier<Double> rightTrig, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX) {
    m_driveTrain = driveTrain;

    m_rightTrig = rightTrig;
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
    double xVelocity = -3.0 * scale(MathUtil.applyDeadband(m_leftY.get(), 0.2), 2.5);
    double yVelocity = -3.0 * scale(MathUtil.applyDeadband(m_leftX.get(), 0.2), 2.5);

    /*
    double adjustedTrig = Math.tan(m_rightTrig.get() * Math.atan(2.5)) / 2.5;

    double angle = Math.atan2(MathUtil.applyDeadband(m_leftY.get(), 0.13), MathUtil.applyDeadband(m_leftX.get(), 0.13));
    double xVelocity = -3.0 * adjustedTrig * Math.sin(angle);
    double yVelocity = -3.0 * adjustedTrig * Math.cos(angle);

    if (Math.abs(m_leftY.get()) < 0.13 && Math.abs(m_leftX.get()) < 0.13) {
      xVelocity = 0;
      yVelocity = 0;
    } else if (adjustedTrig < 0.13) {
      xVelocity = -0.3 * Math.sin(angle);
      yVelocity = -0.3 * Math.cos(angle);
    }

    */

    double omega = 4 * Math.PI * -scale(MathUtil.applyDeadband(m_rightX.get(), 0.2), 5);

    double speed = Math.hypot(xVelocity, yVelocity);
    double newx = -0.01 * speed * omega * xVelocity;
    double newy = -0.01 * speed * omega * yVelocity;
    SmartDashboard.putNumber("newx", newx);
    SmartDashboard.putNumber("newy", newy);

    m_driveTrain.drive(xVelocity, yVelocity, omega); // max speed: 3 m/s transitional, pi rad/s (0.5 rotation/s) rotational (for now)
  }

  private double scale(double in, double scale) {
    // return in;
    return Math.tan(in * Math.atan(scale)) / scale;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted
  ) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
