/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {

	//Talon Setting Constants
	public static final double kFalconMaxRpm = 6000; //Free speed RPM of falcon is ~6300
	public static final double kTickPerRev = 2048; //Encoder ticks per revolution for the Falcon 500
    public static final int kDriveSlotIdx = 0; //Which PID Slot to pull gains from (0,1,2,3)
    public static final int kElevatorSlotIdx = 1; //Which PID Slot to pull gains from (0,1,2,3)
    public static final int kWristSlotIdx = 2;
	public static final int kPIDLoopIdx = 0; //Which Cascaded PID Loop
    public static final int kTimeoutMs = 10; //Set 0 to skip waiting for confirmation

	//Flywheel Constants
	public static final double kFlywheelkP = 0;
	public static final double kFlywheelkI = 0;
	public static final double kFlywheelkD = 0;
	public static final double kFlywheelkF = 0.0505;

	//Drive Constants
	public static final double kRightDrivekP = 0;
	public static final double kRightDrivekI = 0;
	public static final double kRightDrivekD = 0;
	public static final double kRightDrivekF = 0.0500;

	public static final double kLeftDrivekP = 0;
	public static final double kLeftDrivekI = 0;
	public static final double kLeftDrivekD = 0;
	public static final double kLeftDrivekF = 0.0456;

	//Drive Characterization Feed Forward/Feedback Gains
	public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

	public static final double kPDriveVel = 8.5;
	
	//Drive Kinematics
	public static final double kWheelDimaterMeters = 0.1905;
	public static final double kTrackwidthMeters = 0.69;
	public static final DifferentialDriveKinematics kDriveKinematics = 
		new DifferentialDriveKinematics(kTrackwidthMeters);

	public static final double kMaxSpeedMetersPerSecond = 3;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;

	//RAMSETE Parameters
	public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

	//Robot Mapping
    public enum RobotMap {
        //Controller Mapping
	    CONTROLLER_MAIN(0),
	    CONTROLLER_PARTNER(1),
	    //CAN Motors Mapping
	    LEFT_SLAVE(7),
	    LEFT_MASTER(2), 
	    RIGHT_SLAVE(8), 
	    RIGHT_MASTER(1),
		FLYWHEEL_MOTOR(5),
		PHOTOELEC_SENSOR(3),
	    INTAKE_MOTOR(5);
		
		public final int value;

    	RobotMap(int value) {
			this.value = value;
    	}
    }
}
