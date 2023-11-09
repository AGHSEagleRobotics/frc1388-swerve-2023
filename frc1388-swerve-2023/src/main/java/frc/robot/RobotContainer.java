// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDriveCommand;
// import frc.robot.commands.Autos;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveModuleTestCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.SwerveModuleTestSubsystem;

import java.time.Instant;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.*;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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

  /**Limelight */
  private final Limelight m_Limelight = new Limelight();

  /** DriveTrain */
  private final SwerveDriveTrain m_driveTrain = new SwerveDriveTrain(
      new SwerveModule(
          new WPI_TalonFX(1),
          new WPI_TalonFX(5),
          new CANCoder(9),
          282,
          "frontRight"),
      new SwerveModule(
          new WPI_TalonFX(2),
          new WPI_TalonFX(6),
          new CANCoder(10),
          203,
          "frontLeft"),
      new SwerveModule(
          new WPI_TalonFX(3),
          new WPI_TalonFX(7),
          new CANCoder(11),
          36,
          "backLeft"),
      new SwerveModule(
          new WPI_TalonFX(4),
          new WPI_TalonFX(8),
          new CANCoder(12),
          167,
          "backRight"),
      new AHRS(SerialPort.Port.kUSB)
    );
  private final SwerveModule m_swerveModule = new SwerveModule(
    new WPI_TalonFX(1),
    new WPI_TalonFX(5),
    new CANCoder(9),
    71,
    "frontRight");


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
        () -> m_driverController.getRightTriggerAxis(),
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX()
      );
    m_driveTrain.setDefaultCommand(m_swerveCommand);

    m_driverController.a().onTrue(new InstantCommand(() -> m_driveTrain.resetGyroHeading()));
    m_driverController.a().onTrue(new InstantCommand(() -> m_driveTrain.resetPose(new Pose2d())));


    // m_driverController.y().whileTrue(new RunCommand(() -> m_driveTrain.drive(0, 0.6, 0)));
    // m_driverController.a().whileTrue(new RunCommand(() -> m_driveTrain.drive(0, -0.6, 0)));
    m_driverController.x().whileTrue(new RunCommand(() -> m_driveTrain.drive(-0.6, 0, 0)));
    // m_driverController.b().whileTrue(new RunCommand(() -> m_driveTrain.drive(0.6, 0, 0)));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {




    final PathPlannerPath examplePath = PathPlannerPath.fromPathFile("basic");
    m_driveTrain.resetPose(new Pose2d());
    return m_driveTrain.getSwerveControllerCommand(examplePath);

  }
}
