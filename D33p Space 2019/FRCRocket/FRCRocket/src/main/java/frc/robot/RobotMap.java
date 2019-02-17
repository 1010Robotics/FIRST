/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot;

//RobotMap deffinitions
public enum RobotMap 
{

  //Controller Mapping
	CONTROLLER_MAIN(0),
	CONTROLLER_PARTNER(1),
  //CAN Motors Mapping
	LEFT_MOTORF(6),
	LEFT_MOTOR(1),
	LEFT_MOTORF2(7), 
	RIGHT_MOTORF(8), 
	RIGHT_MOTOR(2),
	RIGHT_MOTORF2(9), 
	ELEVATOR_MOTORL(4),
	ELEVATOR_MOTORR(5),
	WRIST_MOTOR(4),
	INTAKE_MOTOR(10),
	//Controllers Mapping
	LEFT_JOYSTICK(0),
	RIGHT_JOYSTICK(1);

	public final int value;

  RobotMap(int value) 
  {
		this.value = value;
  }
  
}
