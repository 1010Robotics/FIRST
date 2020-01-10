/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Talon Setting Constants
    public static final int kDriveSlotIdx = 0; //Which PID Slot to pull gains from (0,1,2,3)
    public static final int kElevatorSlotIdx = 1; //Which PID Slot to pull gains from (0,1,2,3)
    public static final int kWristSlotIdx = 2;
	public static final int kPIDLoopIdx = 0; //Which Cascaded PID Loop
    public static final int kTimeoutMs = 10; //Set 0 to skip waiting for confirmation
    
    public static final int rightSparkPort = 1;
    public static final int leftSparkPort = 2;

    public enum RobotMap 
    {

        //Controller Mapping
	    CONTROLLER_MAIN(0),
	    CONTROLLER_PARTNER(1),
	    //CAN Motors Mapping
	    LEFT_MOTORF2(12),
	    LEFT_MOTORF(7),
	    LEFT_MOTOR(2), 
	    RIGHT_MOTORF2(11), 
	    RIGHT_MOTORF(8), 
	    RIGHT_MOTOR(1),
	    ELEVATOR_MOTOR(4),
	    ELEVATOR_MOTORF(5),
	    INTAKE_MOTOR(5),
	    //Controllers Mapping
	    LEFT_JOYSTICK(0),
	    RIGHT_JOYSTICK(1);

	    public final int value;

     RobotMap(int value) 
    {
		this.value = value;
    }
  
    }
}
