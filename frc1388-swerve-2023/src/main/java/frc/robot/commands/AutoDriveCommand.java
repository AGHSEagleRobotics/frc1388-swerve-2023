// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveTrain;

public class AutoDriveCommand extends CommandBase {
  private final SwerveDriveTrain m_swerveDriveTrain;
  private final Timer m_timer;

  /** Creates a new AutoDriveCommand. */
  public AutoDriveCommand(SwerveDriveTrain driveTrain) {
    m_swerveDriveTrain = driveTrain;
    m_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentTime = m_timer.get();
    // paths
    // line
    // FinalExamplePathBasic
    // Auto_test_path
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Auto_test_path", new PathConstraints(2, 2));
    PathPlannerState state = (PathPlannerState)examplePath.sample(currentTime);
    double xVelocity = state.holonomicRotation.getCos() * state.velocityMetersPerSecond;
    double yVelocity = state.holonomicRotation.getSin() * state.velocityMetersPerSecond;
    double omega = state.angularVelocityRadPerSec;
    System.out.println("x: " +  xVelocity + "\t  y: " + yVelocity + "\t  rotation: " + omega + "\t  time: " + currentTime);
    m_swerveDriveTrain.drive(xVelocity, yVelocity, omega);
    // System.out.println("doing swerve auto, current speed is: " + state.velocityMetersPerSecond + "\t at time: " + m_timer.get() + " seconds");
    // ^^^ this may or may not work
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() > 5); // if the timer is more than 5 seconds, the auto program is done
  }
}
