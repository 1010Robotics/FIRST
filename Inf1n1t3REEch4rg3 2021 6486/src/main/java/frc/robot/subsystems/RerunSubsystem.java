// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.io.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RerunSubsystem extends SubsystemBase {
  /** Creates a new RerunSubsystem. */
  public RerunSubsystem() {}
  public void save(String value,String filename) {
    File f = new File("/home/lvuser/"+filename);
    try {
      //if file does not exist, create one
      FileOutputStream fop = new FileOutputStream(f,true);
      OutputStreamWriter writer = new OutputStreamWriter(fop, "UTF-8");
      writer.append(value);
      writer.append("\r\n");
      writer.close();
      fop.close();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

