/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class turnAngle extends Command {

  private double angle;

  public turnAngle(double angle) {
    requires(Robot.drivebase);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivebase.gyroReset();
    Robot.drivebase.rotateDegrees(angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Gyro Angle", (Robot.drivebase.getGyroPosition()));
    double power = Robot.drivebase.turnController.get();
    //float kp = (float)0.004;
    //double error = angle - Robot.drivebase.getGyroPosition();
    //double power = error * kp;
    Robot.drivebase.set(ControlMode.PercentOutput, (power), -(power));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivebase.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
 
  }
}
