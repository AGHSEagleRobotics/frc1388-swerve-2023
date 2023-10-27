// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

  private final NetworkTableEntry m_tx;
  private final NetworkTableEntry m_ty;
  private final NetworkTableEntry m_ta;


  private final NetworkTableEntry m_botPos;
  private final NetworkTableEntry m_camPos;


  /** Creates a new Limelight. */
  public Limelight() {

    m_tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    m_ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    m_ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta");

    m_botPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
    m_camPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace");

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tx", m_tx.getDouble(0));
    SmartDashboard.putNumber("ty", m_ty.getDouble(0));
    SmartDashboard.putNumber("ta", m_ta.getDouble(0));



    double[] rawPos =  m_botPos.getDoubleArray(new double[6]);
    double[] camPos =  m_camPos.getDoubleArray(new double[6]);

    if (rawPos.length != 0) {
      SmartDashboard.putNumber("x pose-limelight", rawPos[0]);
      SmartDashboard.putNumber("y pose-limelight", rawPos[1]);
      SmartDashboard.putNumber("z pose-limelight", rawPos[2]);
    }

    if (camPos.length != 0) {
      SmartDashboard.putNumber("x cam pose-limelight", camPos[0]);
      SmartDashboard.putNumber("y cam pose-limelight", camPos[1]);
      SmartDashboard.putNumber("z cam pose-limelight", camPos[2]);
    }

    SmartDashboard.putString("XXXXXXXXXXXXXfjasdkljfkdlsa", LimelightHelpers.getCameraPose3d_TargetSpace("limelight").toString());
  }
}
