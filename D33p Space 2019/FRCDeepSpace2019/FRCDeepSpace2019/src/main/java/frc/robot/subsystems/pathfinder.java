/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;


public class pathfinder extends Subsystem {
  
  private Waypoint[] points;
  private Trajectory.Config config;
  private Trajectory trajectory;
  private TankModifier modifier;

	public EncoderFollower leftEnc;
	public EncoderFollower rightEnc;

  public pathfinder(){

    points = new Waypoint[] {
      new Waypoint(0, 0, Pathfinder.d2r(-45)),
      new Waypoint(5, 0, Pathfinder.d2r(-45)) 
     };

    config  = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_CUBIC,
      Trajectory.Config.SAMPLES_HIGH, 
      0.05, 
      1.7, 
      2.0, 
      60.0
    );

    trajectory = Pathfinder.generate(points, config);

    modifier = new TankModifier(trajectory).modify(Robot.drive.baseWidth);

    leftEnc = new EncoderFollower(modifier.getLeftTrajectory());
    rightEnc = new EncoderFollower(modifier.getRightTrajectory());

    leftEnc.configureEncoder(Robot.drive.getLeftPositionRaw(), Robot.drive.talonTPR, Robot.drive.wheelDiameter);
    rightEnc.configureEncoder(Robot.drive.getRightPositionRaw(), Robot.drive.talonTPR, Robot.drive.wheelDiameter);

    leftEnc.configurePIDVA(
      0.0, //kP
      0.0, //kI
      0.0, //kD
      1 / Robot.drive.maxVelocity, //Max Velocity Ratio
      1 //Acceleration Gain
    );
    rightEnc.configurePIDVA(
      0.0, //kP
      0.0, //kI
      0.0, ///kD
      1 / Robot.drive.maxVelocity, //Max Velocity Ratio
      1 //Acceleration Gain
    );

  }

  @Override
  public void initDefaultCommand() {
  }
  
}
