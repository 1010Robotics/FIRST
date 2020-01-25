/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class FollowPath extends CommandBase {
  
  private DriveSubsystem drive;
  private Trajectory traj;
  private TankModifier modifier;
  private EncoderFollower left;
  private EncoderFollower right;

  //Path Follower Constants
	public static final float kP = 1.0f;
  public static final float kI = 0;
  public static final float kD = 0;

  public FollowPath(Trajectory traj) {
    this.traj = traj;
  }               

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    modifier = new TankModifier(traj).modify(Constants.kTrackwidthMeters);

    left = new EncoderFollower(modifier.getLeftTrajectory());
    right = new EncoderFollower(modifier.getRightTrajectory());

    left.configureEncoder((int)drive.getLeftPositionRaw(), Constants.kTickPerRev, Constants.kWheelDimaterMeters);
    right.configureEncoder((int)drive.getRightPositionRaw(), Constants.kTickPerRev, Constants.kWheelDimaterMeters);

    left.configurePIDVA(kP, kI, kD, Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
    right.configurePIDVA(kP, kI, kD, Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lOut = left.calculate((int)drive.getLeftPositionRaw());
    double rOut = right.calculate((int)drive.getRightPositionRaw());
    double targetHeading = Pathfinder.r2d(left.getHeading());

    double diffAngle = Pathfinder.boundHalfDegrees(targetHeading - drive.getAngle());
    diffAngle %= 360.0;
    if(Math.abs(diffAngle) > 180.0) {
      diffAngle = (diffAngle > 0) ? diffAngle - 360 : diffAngle + 360;
    }

    double turn = 0.8 * (-1.0/80.0) * diffAngle;

    drive.set(rOut - turn, lOut + turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
