/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class autoTurn extends Command {

  //PID Constants
  private float kp = 0.007643f;
  private float ki = 0.0f;
  private float kd = 2.50463f;

  //PID  Variables
  private double angle;
  private double error;
  private double errorDiff;
  private double errorSum;
  private double errorLast;
  private double p;
  private double i;
  private double d;
  private double outputPID;

  public autoTurn(double angle) {
    requires(Robot.drive);
    this.angle =  angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.gyroReset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
		SmartDashboard.putNumber("Gyro Angle", (Robot.drive.getGyroPosition()));
    SmartDashboard.putNumber("Turn Error", error);

    error = angle - Robot.drive.getGyroPosition();
    errorLast = error;
    errorDiff = error - errorLast;
    errorSum += error;

    p = kp * error;
    i = ki * errorSum;
    d = kd * errorDiff;
    outputPID  = p + i + d;

    if(outputPID > 1.0){outputPID = 1;}
    if(outputPID < -1.0){outputPID = -1;}

    Robot.drive.pidWrite(outputPID);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
