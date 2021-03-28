/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLightTop extends SubsystemBase {

	//private static String lime;
	private static NetworkTableInstance table = null;

	public enum LightMode {
		eOn, eOff, eBlink
	}

	public enum CameraMode {
		eVision, eDriver
	}

	public boolean isTarget() {
		return getValue("tv").getDouble(0) == 1;
	}

	public double getTx() {
		return getValue("tx").getDouble(0.00);
	}

	public double getTy() {
		return getValue("ty").getDouble(0.00);
	}

	public double getTa() {
		return getValue("ta").getDouble(0.00);
	}

	public double getTs() {
		return getValue("ts").getDouble(0.00);
	}

	public double getTl() {
		return getValue("tl").getDouble(0.00);
	}

	public void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	public void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

	private static NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault();
		}

		return table.getTable("limelight-main").getEntry(key);
  	}

/*   	@Override
 	 public void initDefaultCommand() {
 	} */
  
}
