/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;

public class followPath extends Command {

  public followPath() {
    requires(Robot.drive);
  //  requires(Robot.path);
  }

  @Override
  protected void initialize() {
    Robot.drive.resetEnc();
    Robot.drive.gyroReset();
  }

  @Override
  protected void execute() {
  //  double l = Robot.path.leftEnc.calculate(Robot.drive.getLeftPositionRaw());
  //  double r = Robot.path.rightEnc.calculate(Robot.drive.getRightPositionRaw());

    double gyro_heading = Robot.drive.getGyroPosition();    // Assuming the gyro is giving a value in degrees
    //double desired_heading = Pathfinder.r2d(Robot.path.leftEnc.getHeading());  // Should also be in degrees

   // double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
   // double turn = 0.8 * (-1.0/80.0) * angleDifference;

   // Robot.drive.set(ControlMode.PercentOutput, r-turn, l+turn);//switch +-
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
