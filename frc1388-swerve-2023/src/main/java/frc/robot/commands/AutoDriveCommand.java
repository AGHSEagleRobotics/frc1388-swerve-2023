// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveTrain;

@Deprecated
public class AutoDriveCommand extends CommandBase {
  private final SwerveDriveTrain m_swerveDriveTrain;
  private final Timer m_timer;

  private final double m_x;
  private final double m_y;
  private final double m_rot;


  private boolean isFinished = false;

  private PIDController xPid;
  private PIDController yPid;
  private PIDController rotPid;



  /** Creates a new AutoDriveCommand. */
  public AutoDriveCommand(SwerveDriveTrain driveTrain, double x, double y, double rot) {
    m_swerveDriveTrain = driveTrain;
    m_timer = new Timer();

    m_x = x;
    m_y = y;
    m_rot = rot;

    xPid = new PIDController(1.5, 0, 1);
    yPid = new PIDController(1.5, 0, 1);
    rotPid = new PIDController(0.03, 0.08, 0);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    xPid.reset();
    yPid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX = m_swerveDriveTrain.getPose().getX();
    double currentY = m_swerveDriveTrain.getPose().getY();
    double currentRot = m_swerveDriveTrain.getPose().getRotation().getDegrees();


    if (
      Math.abs(Math.abs(currentX) - Math.abs(m_x)) < 0.1
      && Math.abs(Math.abs(currentY) - Math.abs(m_y)) < 0.1
      && Math.abs(Math.abs(currentRot) - Math.abs(m_rot)) < 0.1) {
      // System.out.println("========================================\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n=\n===================================");
      // System.out.println("x dif" + (Math.abs(currentX) - Math.abs(m_x)));
      // System.out.println("x dif" + (Math.abs(currentY) - Math.abs(m_y)));
      isFinished = true;
      return;
    }

    // double xSpeed = Math.min(xPid.calculate(currentX, m_x), 0.6);
    double xSpeed = MathUtil.clamp(xPid.calculate(currentX, m_x), -0.6, 0.6);
    // double ySpeed = Math.min(yPid.calculate(currentY, m_y), 0.6);
    double ySpeed = MathUtil.clamp(yPid.calculate(currentY, m_y), -0.6, 0.6);
    // double omega = Math.min(rotPid.calculate(currentRot, m_rot), 0.6);
    double omega = MathUtil.clamp(rotPid.calculate(currentRot, m_rot), -2, 2);

    SmartDashboard.putString("X, Y speed", xSpeed + ", " + ySpeed);
    m_swerveDriveTrain.drive(xSpeed, ySpeed, omega);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDriveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (m_timer.get() > 5); // if the timer is more than 5 seconds, the auto program is done
    return isFinished;

  }
}
