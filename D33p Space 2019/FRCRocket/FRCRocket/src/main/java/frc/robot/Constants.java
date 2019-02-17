/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot;

//Creating a public object named "Constants" with properties that set new constants using Gains.java to be used for a PID
public class Constants {

    //Talon Setting Constants
    public static final int kDriveSlotIdx = 0; //Which PID Slot to pull gains from (0,1,2,3)
    public static final int kElevatorSlotIdx = 1; //Which PID Slot to pull gains from (0,1,2,3)
    public static final int kWristSlotIdx = 2; //Which PID Slot to pull gains from (0,1,2,3)
    //PID Constants
    public static final int kPIDLoopIdx = 0; //Which Cascaded PID Loop
	public static final int kTimeoutMs = 10; //Set 0 to skip waiting for confirmation
   
    //PID Wrist Constants
    public static Gains kWristGains = new Gains(0.6, 0.0, 0.0, 0.6);
    
    //PID Elevator Constants
    public static Gains kElevatorGains = new Gains(0.42, 0.0, 0.0, 0.2);

    //PID Drive Constants
    public static Gains kDriveGains = new Gains(0.03, 0.0, 0.0, 0.3);

    //PID Turn Constants
    public static Gains kTurnGains= new Gains(0.007643, 0.0, 2.50463, 0.2);

}
