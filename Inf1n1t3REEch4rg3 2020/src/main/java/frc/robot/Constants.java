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
    public static final int kDriveSlotIdx = 0; //Which PID Slot to pull gains from (0,1,2,3)
    public static final int kElevatorSlotIdx = 1; //Which PID Slot to pull gains from (0,1,2,3)
    public static final int kWristSlotIdx = 2;
	public static final int kPIDLoopIdx = 0; //Which Cascaded PID Loop
    public static final int kTimeoutMs = 10; //Set 0 to skip waiting for confirmation
	
	//Flywheel Constants (to be later removed)
	public static final double kFlywheelkP = 0.00055;
	public static final double kFlywheelkD = 0.0008;

	//Drive Characterization Feed Forward/Feedback Gains
	public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

	public static final double kPDriveVel = 8.5;
	
	//Drive Kinematics
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
	    FLYWHEEL_MOTOR(7),
	    INTAKE_MOTOR(5);
		
		public final int value;

    	RobotMap(int value) {
			this.value = value;
    	}
    }
}
