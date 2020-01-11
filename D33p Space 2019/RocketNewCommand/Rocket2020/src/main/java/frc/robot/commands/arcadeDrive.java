/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.driveBase;

public class arcadeDrive extends CommandBase {

  private final driveBase chassis;
  /**
   * Creates a new arcadeDrive.
   */
  public arcadeDrive(driveBase sub1) {
    chassis = sub1;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Exponential Variables
  private final double JoyDead = 0.050;
  private final double DriveExp = 1.7;
  private final double MotorMin = 0.008;
   
  //Align Code
	private float moveKp = 0.0075f;
	private float trueKp = 0.008f;  

  
  //Joystick OI Variables
	private boolean count;
	private double trueAngle = 0;
	private double correction;
	private double kPerr;
	private double joyYval;
	private double joyXval;
	private double yOutput;
  private double xOutput;
  
  // Exponential Function
  private double exponential(double joystickVal, double driveExp, double joyDead, double motorMin) {
     double joySign;
     double joyMax = 1 - joyDead;
     double joyLive = Math.abs(joystickVal) - joyDead;
     if (joystickVal > 0) {
       joySign = 1;
     } else if (joystickVal < 0) {
       joySign = -1;
     } else {
       joySign = 0;
     }
     double power = (joySign
         * (motorMin + ((1 - motorMin) * (Math.pow(joyLive, driveExp) / Math.pow(joyMax, driveExp)))));
     if (Double.isNaN(power)) {
       power = 0;
     }
     return power;
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.oi.main.getXButton()) {

		}
		else {
			joyYval = Robot.oi.main.getY(Hand.kLeft);
			joyXval = Robot.oi.main.getX(Hand.kRight);

			yOutput = exponential(joyYval, DriveExp, JoyDead, MotorMin);
			xOutput = exponential(joyXval, DriveExp, JoyDead, MotorMin);

			//Update Closed Loop Variables
			if((Math.abs(yOutput) > JoyDead)&& (Math.abs(xOutput) > JoyDead)){
				correction = 0; 
			}
			else{
				if(Math.abs(xOutput) > JoyDead){
					count = false; 
				}
				else{
					if(Math.abs(yOutput) > JoyDead){
						if(count == false){
							trueAngle = chassis.getGyroPosition();
							count = true;
						}
						kPerr = trueAngle - chassis.getGyroPosition();
						correction = kPerr * trueKp;
					}
				}
			}

			SmartDashboard.putNumber("True Gyro Variable", trueAngle);
			SmartDashboard.putBoolean("Count Variable", count);
			SmartDashboard.putNumber("Gyro Position", chassis.getGyroPosition());
			SmartDashboard.putNumber("KP Error", kPerr);
			SmartDashboard.putNumber("Correction", correction);
			SmartDashboard.putNumber("Left Pos", chassis.getLeftPositionRaw());
			SmartDashboard.putNumber("Right Pos", chassis.getRightPositionRaw());


			chassis.set(ControlMode.PercentOutput, ((yOutput) + xOutput), ((yOutput) - xOutput));
/* 
			gyroOutput.setNumber(chassis.getGyroPosition());

			JoystickOutput_X.setNumber(xOutput);
			JoystickOutput_Y.setNumber(yOutput); */
		}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
