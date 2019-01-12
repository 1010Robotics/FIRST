/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.teleopCamera;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLight extends Subsystem {

  private static NetworkTableInstance table;
  
  @Override
  public void initDefaultCommand() {
     setDefaultCommand(new teleopCamera());
  }

  public limeLight(){
    table = null;
    SmartDashboard.putBoolean("limeLight Active", true);
  }
  
  private static NetworkTableEntry getValue(String key){
    if(table == null){
      table = NetworkTableInstance.getDefault();
    }
    return table.getTable("limelight").getEntry(key);
  }

  public static enum CameraLightMode {
    light_On, light_Off, light_Blink
  }
  public static enum CameraMode {
    mode_Vision, mode_Driver
  }
  public static boolean isTarget(){
    return getValue("tv").getDouble(0) == 1;
  }
  public static double getTy(){
    return getValue("ty").getDouble(0.00);
  }
  public double getTx(){
    return getValue("tx").getDouble(0.00);
  }
  public static double getTa(){
    return getValue("ta").getDouble(0.00);
  }
  public static double getTs(){
    return getValue("tl").getDouble(0.00);
  }
  public static void setLedMode(CameraLightMode mode){
    getValue("ledMode").setNumber(mode.ordinal());
  }
  public static void setCameraMode(CameraMode mode){
    getValue("camMode").setNumber(mode.ordinal());
  }
  public static void setPipeline(int value){
    getValue("pipeline").setNumber(value);
  }
}
