/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.driveBase;
import frc.robot.commands.turnAngle;
import frc.robot.commands.Drive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.*;


public class Robot extends IterativeRobot {



	//Constants
	public static final int kSlotIdx = 0; //Which PID Slot to pull gains from (0,1,2,3)
	public static final int kPIDLoopIdx = 0; //Which Cascaded PID Loop
	public static final int kTimeoutMs = 10; //Set 0 to skip waiting for c<onfirmation

	public static OI oi;
	public static driveBase drivebase;
	
	Command autonomousCommand;
	
	/*@SuppressWarnings("rawtypes")
	SendableChooser autoSelector;*/


	public enum RobotState {
        DISABLED, AUTONOMOUS, TELEOP
    }
	
    public static RobotState s_robot_state = RobotState.DISABLED;

    public static RobotState getState() {
        return s_robot_state;
    }

    public static void setState(RobotState state) {
        s_robot_state = state;
    }

	@Override
	public void robotInit() {
		//create new objects
		oi = new OI();
		drivebase = new driveBase();
		autonomousCommand = new Drive(3);
	//	autoSelector = new SendableChooser();
		SmartDashboard.putData(drivebase);
	
	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand.start();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override  
	public void testPeriodic() {
		
	}
	
	public static void initTalon(TalonSRX motor, boolean invert) {
		motor.setInverted(invert);
		motor.setNeutralMode(NeutralMode.Coast);
		motor.neutralOutput();
		motor.setSensorPhase(false);
		motor.configNominalOutputForward(0.0, 0);
		motor.configNominalOutputReverse(0.0, 0);
		motor.configClosedloopRamp(0.5, 0);
		motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	}
	public static void initVictor(VictorSPX motor, boolean invert) {
		motor.setInverted(invert);
		motor.setNeutralMode(NeutralMode.Coast);
		motor.neutralOutput();
		motor.setSensorPhase(false);
		motor.configNominalOutputForward(0.0, 0);
		motor.configNominalOutputReverse(0.0, 0);
		motor.configClosedloopRamp(0.5, 0);
	}

	public static void initMasterDriveMotor(TalonSRX motor){
		motor.configNominalOutputForward(0, kTimeoutMs);
		motor.configNominalOutputReverse(0, kTimeoutMs);
		motor.configPeakOutputForward(1, kTimeoutMs);
		motor.configPeakOutputReverse(-1, kTimeoutMs);
		motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		motor.config_kF(0, 0.2, kTimeoutMs);
		motor.config_kP(0, 0.2, kTimeoutMs);
		motor.config_kI(0, 0, kTimeoutMs);
		motor.config_kD(0, 0, kTimeoutMs);
		motor.configMotionCruiseVelocity(15000, kTimeoutMs);
		motor.configMotionAcceleration(6000, kTimeoutMs);
		motor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
	}
}
