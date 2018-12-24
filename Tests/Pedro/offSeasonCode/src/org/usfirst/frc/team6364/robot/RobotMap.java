/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6364.robot;

public enum RobotMap {

	//CAN Motors Mapping
	LEFT_MOTORF(0),
	LEFT_MOTOR(1), 
	RIGHT_MOTORF(2), 
	RIGHT_MOTOR(3),
	//Controllers Mapping
	LEFT_JOYSTICK(0),
	RIGHT_JOYSTICK(1);

	public final int value;

	RobotMap(int value) {
		this.value = value;
	}
}
