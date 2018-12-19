/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
	public static final int LEFT_HZ_AXIS = 0;
	public static final int LEFT_VT_AXIS = 1;
	public static final int LEFT_Z_AXIS = 3;
	public static final int RIGHT_HZ_AXIS = 4;
	public static final int RIGHT_VT_AXIS = 5;
	public static final int RIGHT_Z_AXIS = 2;
	public static final double Trigger_deadZone = 0.0;//tbd
	public static final double Trigger_MaxVal = 1.0;//tbd
	public static final double deadZone = 0.1;
	public static final double MaxVal = 0.97;

	public static final double JOY_DEADZONE = 0.05;

	public final Joystick LEFT_JOY = new Joystick(RobotMap.LEFT_JOYSTICK.value);
	public final Button DriverButtonA = new JoystickButton(LEFT_JOY, 1);
	public final Button DriverButtonB = new JoystickButton(LEFT_JOY, 2);
	public final Button DriverButtonX = new JoystickButton(LEFT_JOY, 3);
	public final Button DriverButtonY = new JoystickButton(LEFT_JOY, 4);
	public final Button DriverButtonL = new JoystickButton(LEFT_JOY, 5);
	public final Button DriverButtonR = new JoystickButton(LEFT_JOY, 6);
	public final Button DriverButton_BACK = new JoystickButton(LEFT_JOY, 7);
	public final Button DriverButton_START = new JoystickButton(LEFT_JOY, 8);
	public final Button DriverLeftStickPress = new JoystickButton(LEFT_JOY, 9);
	public final Button DriverRightStickPress = new JoystickButton(LEFT_JOY, 10);
	
	public final Joystick RIGHT_JOY = new Joystick(RobotMap.RIGHT_JOYSTICK.value);
	public final Button OperatorButtonA = new JoystickButton(RIGHT_JOY, 1);
	public final Button OperatorButtonB = new JoystickButton(RIGHT_JOY, 2);
	public final Button OperatorButtonX = new JoystickButton(RIGHT_JOY, 3);
	public final Button OperatorButtonY = new JoystickButton(RIGHT_JOY, 4);
	public final Button OperatorButtonL = new JoystickButton(RIGHT_JOY, 5);
	public final Button OperatorButtonR = new JoystickButton(RIGHT_JOY, 6);
	public final Button OperatorButton_BACK = new JoystickButton(RIGHT_JOY, 7);
	public final Button OperatorButton_START = new JoystickButton(RIGHT_JOY, 8);
	public final Button OperatorLeftStickPress = new JoystickButton(RIGHT_JOY, 9);
	public final Button OperatorRightStickPress = new JoystickButton(RIGHT_JOY, 10);

	public double getLeftJoyX() {
		double raw = LEFT_JOY.getX();
		return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
	}

	public double getLeftJoyY() {
		double raw = LEFT_JOY.getY();
		return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
	}

	public double getRightJoyX() {
		double raw = RIGHT_JOY.getX();
		return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
	}

	public double getRightJoyY() {
		double raw = RIGHT_JOY.getY();
		return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
	}

	public double leftArcade = Robot.oi.getLeftJoyY() + Robot.oi.getRightJoyX();
	public double rightArcade = Robot.oi.getLeftJoyY() - Robot.oi.getRightJoyX();

	public double GetLeftTrigger(int stick) {
		switch (stick) {
		case 0:
			return LEFT_JOY.getRawAxis(LEFT_Z_AXIS);
		case 1:
			return RIGHT_JOY.getRawAxis(LEFT_Z_AXIS);
		default:
			return 0.0;
		}
	}

	public double GetRightTrigger(int stick) {
		double value;
		switch (stick) {
		case 0:
			value = LEFT_JOY.getRawAxis(RIGHT_Z_AXIS);
			return (Math.abs(value) < Trigger_deadZone) ? 0.0 : (Math.abs(value) > Trigger_MaxVal) ? 1.0 : value ; 
		case 1:
			value = RIGHT_JOY.getRawAxis(RIGHT_Z_AXIS);
			return (Math.abs(value) < Trigger_deadZone) ? 0.0 : (Math.abs(value) > Trigger_MaxVal) ? 1.0 : value ; 
		default:
			return 0.0;
		}
	}

	public boolean GetButton(Button button) {
		return button.get();
	}
	public OI() {

	}
}
