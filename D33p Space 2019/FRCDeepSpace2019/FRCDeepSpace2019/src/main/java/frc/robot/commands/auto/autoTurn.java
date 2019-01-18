/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class autoTurn extends Command {

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
    this.angle = angle;
  }

  @Override
  protected void initialize() {
    Robot.drive.gyroReset();
  }

  @Override
  protected void execute() {
    
		SmartDashboard.putNumber("Gyro Angle", (Robot.drive.getGyroPosition()));
    SmartDashboard.putNumber("Turn Error", error);

    error = angle - Robot.drive.getGyroPosition();
    errorLast = error;
    errorDiff = error - errorLast;
    errorSum += error;

    p = Constants.kTurnGains.kP * error;
    i = Constants.kTurnGains.kI * errorSum;
    d = Constants.kTurnGains.kD * errorDiff;
    outputPID  = p + i + d;

    if(outputPID > 1.0){outputPID = 1;}
    if(outputPID < -1.0){outputPID = -1;}

    Robot.drive.pidWrite(outputPID);

    try { TimeUnit.MILLISECONDS.sleep(10); } 	
    catch (Exception e) { /* Do Nothing */ }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.drive.stop();
  }

  @Override
  protected void interrupted() {
  }
}
