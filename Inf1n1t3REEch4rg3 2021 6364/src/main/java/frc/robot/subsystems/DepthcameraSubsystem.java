// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.intel.rs.frame.DepthFrame;
import org.intel.rs.frame.FrameList;
import org.intel.rs.pipeline.Config;
import org.intel.rs.pipeline.Pipeline;
import org.intel.rs.pipeline.PipelineProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DepthcameraSubsystem extends SubsystemBase {
  private Pipeline pipeline = new Pipeline();
  private DepthFrame depth;
  private float distToCenter;
  private int width;
  private int height;
  private FrameList frames;
  /** Creates a new DepthcameraSubsystem. */
  public DepthcameraSubsystem() {}

  public void setup() {
    pipeline.start();
  }

  public void getDistance(){
    frames = pipeline.waitForFrames();
    depth = frames.getDepthFrame();
    width = depth.getWidth();
    height = depth.getHeight();
    distToCenter = depth.getDistance(width / 2, height / 2);
  }

  public void stop() {
    pipeline.stop();
  }

  public void test(){
    SmartDashboard.putNumber("Distance from D435i", distToCenter);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}