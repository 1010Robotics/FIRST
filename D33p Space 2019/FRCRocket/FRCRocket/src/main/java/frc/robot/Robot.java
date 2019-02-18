/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot;

import frc.robot.commands.auto.followPath;
import frc.robot.subsystems.driveBase;
import frc.robot.subsystems.elevatorBase;
import frc.robot.subsystems.intakeBase;
import frc.robot.subsystems.limeLight;
import frc.robot.subsystems.pathfinder;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.wristBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.*;

//Creates a public object named "Robot" which is part of TimedRobot with properties that handles game information and initialization
public class Robot extends TimedRobot {

	//Defines auto selector definitions based on game data
	DriverStation.Alliance colour;
	private boolean isBlue;

	//Create all objects
	public static OI oi;
	public static driveBase drive;
	public static limeLight camera;
	public static elevatorBase elevator;
	public static pneumatics solenoid;
	public static pathfinder path;
	public static wristBase wrist;
	public static intakeBase intake;

	//Create all commands
	Command autonomousCommand;
	Command arcadeDrive;
	Command teleopSolenoid;
	Command dashboardCommand;

	/*@SuppressWarnings("rawtypes")
	SendableChooser autoSelector;*/

	//Sets all possible states of RobotState
	public enum RobotState {
        DISABLED, AUTONOMOUS, TELEOP
    }
	
	//Sets the RobotState known as "s_robot_state" as RobotState.DISABLED
    public static RobotState s_robot_state = RobotState.DISABLED;

	//Sets the RobotState state to returns s_robot_state
    public static RobotState getState() {
        return s_robot_state;
    }

	//Sets the state for RobotState, used in initialization 
    public static void setState(RobotState state) {
        s_robot_state = state;
    }

	@Override
	//Initializes robot, contains vital definitions
	public void robotInit() {

		//Defines camera and begins capture
		CameraServer driverMemeCamera = CameraServer.getInstance();
		driverMemeCamera.startAutomaticCapture();

		//Defines all ojects
		oi = new OI();
		drive = new driveBase();
		solenoid = new pneumatics();
		camera = new limeLight();
		path =  new pathfinder();
		elevator = new elevatorBase();
		wrist = new wristBase();
		intake = new intakeBase();

		//Auto selector
		//autoSelector = new SendableChooser();
		//Creates string finding either: DriverStation.Alliance.Blue or DriverStation.Alliance.Red
		colour = DriverStation.getInstance().getAlliance();
		//Creates bool to find, from game information, if the alliance is blue
		isBlue = (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue);

		//SmartDashboard subsystem initialization
		SmartDashboard.putData(drive);
		SmartDashboard.putData(camera);
		SmartDashboard.putData(solenoid);
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
		isBlue = (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue);
	if(isBlue){/*run a command*/}
		//Set Autonomous Command
		autonomousCommand = new followPath();
		autonomousCommand.start();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		//Stop autonomous command
		//autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override  
	public void testPeriodic() {
	}
	
	//Initializes the talon motors with the given properties
	public static void initTalon(TalonSRX motor, boolean invert) {
		motor.setInverted(invert);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.neutralOutput();
		motor.configNominalOutputForward(0.0, 0);
		motor.configNominalOutputReverse(0.0, 0);
		motor.configClosedloopRamp(0.5, 0);
		motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
	//Initializes the victor motors with the given properties
	public static void initVictor(VictorSPX motor, boolean invert) {
		motor.setInverted(invert);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.neutralOutput();
		motor.configNominalOutputForward(0.0, 0);
		motor.configNominalOutputReverse(0.0, 0);
		motor.configClosedloopRamp(0.5, 0);
	}

	//Initializes the talon drive motors with the given settings
	public static void initMasterDriveMotor(TalonSRX motor){
		motor.setSensorPhase(true);
		//Brake mode
		motor.setNeutralMode(NeutralMode.Brake);
		//Output settings
		motor.configNominalOutputForward(0, Constants.kTimeoutMs);
		motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motor.configPeakOutputForward(1, Constants.kTimeoutMs);
		motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		//PID gain settings
		motor.selectProfileSlot(Constants.kDriveSlotIdx, Constants.kPIDLoopIdx);
		motor.config_kF(Constants.kDriveSlotIdx, Constants.kDriveGains.kF, Constants.kTimeoutMs);
		motor.config_kP(Constants.kDriveSlotIdx, Constants.kDriveGains.kP, Constants.kTimeoutMs);
		motor.config_kI(Constants.kDriveSlotIdx, Constants.kDriveGains.kI, Constants.kTimeoutMs);
		motor.config_kD(Constants.kDriveSlotIdx, Constants.kDriveGains.kD, Constants.kTimeoutMs);
		//Motion magic max velocity and acceleration
		motor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		motor.configMotionAcceleration(6000, Constants.kTimeoutMs);
		//Reset encoder
		motor.setSelectedSensorPosition(0);
	}

	//Initializes the talon elevator motors with the given settings
	public static void initMasterElevatorMotor(TalonSRX motor){
		//Set sensor phase
		motor.setSensorPhase(true);
		//Brake mode
		motor.setNeutralMode(NeutralMode.Brake);
		//Factory default hardware to prevent unexpected behavior
		motor.configFactoryDefault();
		//Set relevant frame periods to be at least as fast as periodic rate
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		//Output settings
		motor.configNominalOutputForward(0, Constants.kTimeoutMs);
		motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motor.configPeakOutputForward(1, Constants.kTimeoutMs);
		motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		//PID gain settings
		motor.selectProfileSlot(Constants.kElevatorSlotIdx, Constants.kPIDLoopIdx);
		motor.config_kF(Constants.kElevatorSlotIdx, Constants.kElevatorGains.kF, Constants.kTimeoutMs);
		motor.config_kP(Constants.kElevatorSlotIdx, Constants.kElevatorGains.kP, Constants.kTimeoutMs);
		motor.config_kI(Constants.kElevatorSlotIdx, Constants.kElevatorGains.kI, Constants.kTimeoutMs);
		motor.config_kD(Constants.kElevatorSlotIdx, Constants.kElevatorGains.kD, Constants.kTimeoutMs);
		//Motion magic max velocity and acceleration
		motor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		motor.configMotionAcceleration(6000, Constants.kTimeoutMs);
		//Reset encoder
		motor.setSelectedSensorPosition(0);
	}

	//Initializes the talon wrist motors with the given settings
	public static void initMasterWristMotor(TalonSRX motor){
		//Set sensor phase
		motor.setSensorPhase(false);
		//Brake mode
		motor.setNeutralMode(NeutralMode.Brake);
		//Factory default hardware to prevent unexpected behavior
		motor.configFactoryDefault();
		//Set relevant frame periods to be at least as fast as periodic rate
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		//Output settings
		motor.configNominalOutputForward(0, Constants.kTimeoutMs);
		motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motor.configPeakOutputForward(1, Constants.kTimeoutMs);
		motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		//PID gain settings
		motor.selectProfileSlot(Constants.kWristSlotIdx, Constants.kPIDLoopIdx);
		motor.config_kF(Constants.kWristSlotIdx, Constants.kWristGains.kF, Constants.kTimeoutMs);
		motor.config_kP(Constants.kWristSlotIdx, Constants.kWristGains.kP, Constants.kTimeoutMs);
		motor.config_kI(Constants.kWristSlotIdx, Constants.kWristGains.kI, Constants.kTimeoutMs);
		motor.config_kD(Constants.kWristSlotIdx, Constants.kWristGains.kD, Constants.kTimeoutMs);
		//Motion magic max velocity and acceleration
		motor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		motor.configMotionAcceleration(6000, Constants.kTimeoutMs);
		//Reset encoder
		motor.setSelectedSensorPosition(0);
	}
}
