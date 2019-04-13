/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class followPath extends Command {

  //Pathfinder
  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  
  private Notifier m_follower_notifier;

  public followPath() {
    requires(Robot.drive);
  }


	//Follow Path
	public void followPathFunction(){
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(Robot.drive.getLeftPositionRaw());
      double right_speed = m_right_follower.calculate(Robot.drive.getRightPositionRaw());
      double heading = Robot.drive.getGyroPosition();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      Robot.drive.set(ControlMode.PercentOutput, right_speed - turn, left_speed + turn);
    }
  }

  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.resetEnc();
    Robot.drive.gyroReset();

    Trajectory left_trajectory = PathfinderFRC.getTrajectory("/home/lvuser/deploy/paths/ToRocket.right.pf1.csv");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory("/home/lvuser/deploy/paths/ToRocket.left.pf1.csv");

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(Robot.drive.getLeftPositionRaw(), Robot.drive.talonTPR, Robot.drive.wheelDiameter);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / Robot.drive.maxVelocity, 0);

    m_right_follower.configureEncoder(Robot.drive.getRightPositionRaw(), Robot.drive.talonTPR, Robot.drive.wheelDiameter);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / Robot.drive.maxVelocity, 0);
    
    m_follower_notifier = new Notifier(this::followPathFunction);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
  }
 
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_follower_notifier.stop();
    Robot.drive.set(ControlMode.PercentOutput, 0, 0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
