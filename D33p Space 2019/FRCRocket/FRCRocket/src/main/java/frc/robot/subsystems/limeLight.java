/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//Creating a public object named "limeLight" which is a Subsystem with properties for getting values fromt he limelight sensor
public class limeLight extends Subsystem {

	//Creates a private (local) unchanging table to be null
	private static NetworkTableInstance table = null;

	//Defines public possible states of "LightMode" 
	public enum LightMode {
		eOn, eOff, eBlink
	}

	//Defines public possible states of "CameraMode" 
	public enum CameraMode {
		eVision, eDriver
	}

	//Creates public bool named "isTarget" that returns true or false
	public boolean isTarget() {
		return getValue("tv").getDouble(0) == 1;
	}

	//Creates public double that returns the value of "tx"
	public double getTx() {
		return getValue("tx").getDouble(0.00);
	}

	//Creates public double that returns the value of "ty"
	public double getTy() {
		return getValue("ty").getDouble(0.00);
	}

	//Creates public double that returns the value of "ta"
	public double getTa() {
		return getValue("ta").getDouble(0.00);
	}

	//Creates public double that returns the value of "ts"
	public double getTs() {
		return getValue("ts").getDouble(0.00);
	}

	//Creates public double that returns the value of "t1"
	public double getTl() {
		return getValue("tl").getDouble(0.00);
	}

	//Creates public function that changes the mode of the LED
	public void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	//Creates public function that changes mode of the camera
	public void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	//Creates public function that changes value of pipeline
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

	//Checks a private unchanging table for a value, where if the table is null it sets the default of "NetworkTableInstance" to "table"
	private static NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault();
		}
		//Returns the value from a table that was given earlier
		return table.getTable("limelight-tenton").getEntry(key);
  	}

  	@Override
 	 public void initDefaultCommand() {
 	}
  
}
