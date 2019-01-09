/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public enum RobotMap 
{

  //Controller Mapping
	CONTROLLER_MAIN(0),
	CONTROLLER_PARTNER(1),
  //CAN Motors Mapping
	LEFT_MOTORF(7),
	LEFT_MOTOR(1), 
	RIGHT_MOTORF(8), 
	RIGHT_MOTOR(2),
	//Controllers Mapping
	LEFT_JOYSTICK(0),
	RIGHT_JOYSTICK(1);

	public final int value;

  RobotMap(int value) 
  {
		this.value = value;
  }
  
}
