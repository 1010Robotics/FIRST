/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot;

//Creates a public object named "Gains" with properties that sets doubles to specific values (used in Constants.java)
public class Gains {
	//Defines the following variables as unchanging doubles
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	
	//Creates a public function that sets specific values for gains
	public Gains(double _kP, double _kI, double _kD, double _kF){
		//Defines the following variables as their inputted counterpart 
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
	}
}
