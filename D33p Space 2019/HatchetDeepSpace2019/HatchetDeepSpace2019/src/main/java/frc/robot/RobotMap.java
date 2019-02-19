/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Talon;

public enum RobotMap 
{

  //Controller Mapping
	CONTROLLER_MAIN(0),
	CONTROLLER_PARTNER(1),
  //CAN Motors Mapping
	LEFT_MOTORF(7),
	LEFT_MOTORF2(8),
	LEFT_MOTOR(5), 
	RIGHT_MOTORF(4), 
	RIGHT_MOTORF2(3), 
	RIGHT_MOTOR(2), 
	ELEVATOR(1),
	INTAKE(6),
	elevatorWheel(8);

	private final int value;

  RobotMap(int value) 
  {

	

		this.value = value;
  }


	public int getValue() {
		return this.value;
	}


	public static void initTalon(Talon elevator2, boolean b) {
	}


  
}
