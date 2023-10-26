// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.helpers.Point3d;

public class Limelight extends SubsystemBase {

  private final NetworkTableEntry m_botPos;
  private final NetworkTableEntry m_camPos;


  /** Creates a new Limelight. */
  public Limelight() {
    // m_botPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace");
    m_botPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");

    m_camPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace");
  }

  // public Point3d getRobotPos() {
  //   double[] rawPos =  m_botPos.getDoubleArray(new double[6]);
  //   return new Point3d(rawPos[0], rawPos[1], rawPos[2]);
    
  // }

  @Override
  public void periodic() {
    double[] rawPos =  m_botPos.getDoubleArray(new double[6]);
    double[] camPos =  m_camPos.getDoubleArray(new double[6]);



    if (rawPos.length != 0) {
      SmartDashboard.putNumber("x pose-limelight", rawPos[0]);
      SmartDashboard.putNumber("y pose-limelight", rawPos[1]);
      SmartDashboard.putNumber("z pose-limelight", rawPos[2]);

      SmartDashboard.putNumber("x cam pose-limelight", camPos[0]);
      SmartDashboard.putNumber("y cam pose-limelight", camPos[1]);
      SmartDashboard.putNumber("z cam pose-limelight", camPos[2]);
    }

    SmartDashboard.putString("XXXXXXXXXXXXXfjasdkljfkdlsa", LimelightHelpers.getCameraPose3d_TargetSpace("limelight").toString());
  }
}
