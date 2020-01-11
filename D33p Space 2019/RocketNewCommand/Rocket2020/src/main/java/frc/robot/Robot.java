/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;
  public static OI oi;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public enum RobotState {
    DISABLED, AUTONOMOUS, TELEOP
  }

  public static RobotState s_robot_state = RobotState.DISABLED;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
	m_robotContainer = new RobotContainer();
	oi = new OI();
	
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  public static void initTalon(TalonSRX motor, boolean invert) {
		motor.setInverted(invert);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.neutralOutput();
		motor.configNominalOutputForward(0.0, 0);
		motor.configNominalOutputReverse(0.0, 0);
		motor.configClosedloopRamp(0.5, 0);
		motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
	public static void initTalonLimitSwitch(TalonSRX motor){

	}
	public static void initVictor(VictorSPX motor, boolean invert) {
		motor.setInverted(invert);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.neutralOutput();
		motor.configNominalOutputForward(0.0, 0);
		motor.configNominalOutputReverse(0.0, 0);
		motor.configClosedloopRamp(0.5, 0);
	}

	public static void initMasterDriveMotor(TalonSRX motor){
		motor.setSensorPhase(true);
		//Brake Mode
		motor.setNeutralMode(NeutralMode.Brake);
		//Output Settings
		motor.configNominalOutputForward(0, Constants.kTimeoutMs);
		motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motor.configPeakOutputForward(1, Constants.kTimeoutMs);
		motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		//PID Gain Settings
		motor.selectProfileSlot(Constants.kDriveSlotIdx, Constants.kPIDLoopIdx);
		motor.config_kF(Constants.kDriveSlotIdx, Constants.kDriveGains.kF, Constants.kTimeoutMs);
		motor.config_kP(Constants.kDriveSlotIdx, Constants.kDriveGains.kP, Constants.kTimeoutMs);
		motor.config_kI(Constants.kDriveSlotIdx, Constants.kDriveGains.kI, Constants.kTimeoutMs);
		motor.config_kD(Constants.kDriveSlotIdx, Constants.kDriveGains.kD, Constants.kTimeoutMs);
		//Motion Magic Max Velocity and Acceleration
		motor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		motor.configMotionAcceleration(6000, Constants.kTimeoutMs);
		//Reset Encoder
		motor.setSelectedSensorPosition(0);
	}

	public static void initMasterElevatorMotor(TalonSRX motor){
		//Set Sensor Phase
		motor.setSensorPhase(false);
		//Brake Mode
		motor.setNeutralMode(NeutralMode.Brake);
		//Factory default hardware to prevent unexpected behavior
		motor.configFactoryDefault();
		//Set relevant frame periods to be at least as fast as periodic rate
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		//Output Settings
		motor.configNominalOutputForward(0, Constants.kTimeoutMs);
		motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motor.configPeakOutputForward(1, Constants.kTimeoutMs);
		motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		//PID Gain Settings
		motor.selectProfileSlot(Constants.kElevatorSlotIdx, Constants.kPIDLoopIdx);
		motor.config_kF(Constants.kElevatorSlotIdx, Constants.kElevatorGains.kF, Constants.kTimeoutMs);
		motor.config_kP(Constants.kElevatorSlotIdx, Constants.kElevatorGains.kP, Constants.kTimeoutMs);
		motor.config_kI(Constants.kElevatorSlotIdx, Constants.kElevatorGains.kI, Constants.kTimeoutMs);
		motor.config_kD(Constants.kElevatorSlotIdx, Constants.kElevatorGains.kD, Constants.kTimeoutMs);
		//Motion Magic Max Velocity and Acceleration
		motor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		motor.configMotionAcceleration(6000, Constants.kTimeoutMs);
		//Reset Encoder
		motor.setSelectedSensorPosition(0);
	}
	public static void initMasterWristMotor(TalonSRX motor){
		//Set Sensor Phase
		motor.setSensorPhase(false);
		//Brake Mode
		motor.setNeutralMode(NeutralMode.Brake);
		//Factory default hardware to prevent unexpected behavior
		motor.configFactoryDefault();
		//Set relevant frame periods to be at least as fast as periodic rate
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		//Output Settings
		motor.configNominalOutputForward(0, Constants.kTimeoutMs);
		motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motor.configPeakOutputForward(0.75, Constants.kTimeoutMs);
		motor.configPeakOutputReverse(-0.75, Constants.kTimeoutMs);
		//PID Gain Settings
		motor.selectProfileSlot(Constants.kElevatorSlotIdx, Constants.kPIDLoopIdx);
		motor.config_kF(Constants.kWristSlotIdx, Constants.kWristGains.kF, Constants.kTimeoutMs);
		motor.config_kP(Constants.kWristSlotIdx, Constants.kWristGains.kP, Constants.kTimeoutMs);
		motor.config_kI(Constants.kWristSlotIdx, Constants.kWristGains.kI, Constants.kTimeoutMs);
		motor.config_kD(Constants.kWristSlotIdx, Constants.kWristGains.kD, Constants.kTimeoutMs);
		//Motion Magic Max Velocity and Acceleration
		motor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		motor.configMotionAcceleration(6000, Constants.kTimeoutMs);
		//Reset Encoder
		motor.setSelectedSensorPosition(0);
	}
  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
   /*  m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } */
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
