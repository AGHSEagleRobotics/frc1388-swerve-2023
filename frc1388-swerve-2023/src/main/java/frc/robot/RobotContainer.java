// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDriveCommand;
// import frc.robot.commands.Autos;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveModuleTestCommand;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.SwerveModuleTestSubsystem;

import java.time.Instant;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /** DriveTrain */
  private final SwerveDriveTrain m_driveTrain = new SwerveDriveTrain(
      new SwerveModule(
          new WPI_TalonFX(1),
          new WPI_TalonFX(5),
          new CANCoder(9),
          71,
          "frontRight"),
      new SwerveModule(
          new WPI_TalonFX(2),
          new WPI_TalonFX(6),
          new CANCoder(10),
          295,
          "frontLeft"),
      new SwerveModule(
          new WPI_TalonFX(3),
          new WPI_TalonFX(7),
          new CANCoder(11),
          343,
          "backLeft"),
      new SwerveModule(
          new WPI_TalonFX(4),
          new WPI_TalonFX(8),
          new CANCoder(12),
          298,
          "backRight"),
      new ADIS16470_IMU());

  /** controller */
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    SwerveDriveCommand m_swerveCommand = new SwerveDriveCommand(
        m_driveTrain,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX()
      );
    m_driveTrain.setDefaultCommand(m_swerveCommand);

    // m_driverController.a().onTrue(new InstantCommand(() -> m_driveTrain.resetGyroHeading()));

    // m_driverController.y().whileTrue(new InstantCommand(() -> m_driveTrain.drive(0, 3, 0)));
    // m_driverController.a().whileTrue(new InstantCommand(() -> m_driveTrain.drive(0, -3, 0)));
    // m_driverController.x().whileTrue(new InstantCommand(() -> m_driveTrain.drive(-3, 0, 0)));
    // m_driverController.b().whileTrue(new InstantCommand(() -> m_driveTrain.drive(3, 0, 0)));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return null; // <- if this is broken, this is why XXX
    return new AutoDriveCommand(m_driveTrain);
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem); XXX example, remove latter

  }
}
