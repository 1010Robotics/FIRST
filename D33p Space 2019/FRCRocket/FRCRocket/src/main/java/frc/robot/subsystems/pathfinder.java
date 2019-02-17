/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

//Creating a public object named "pathfinder" which is a Subsystem with properties that sets and follows a path
public class pathfinder extends Subsystem {
  
  //Defines private defiinitons
  private Waypoint[] points;
  private Trajectory.Config config;
  private Trajectory trajectory;
  private TankModifier modifier;
  //Defines public defiinitons
	public EncoderFollower leftEnc;
	public EncoderFollower rightEnc;

  //Creates public pathfinder function
  public pathfinder(){

    //Creates points to be a new waypiont???
    points = new Waypoint[] {
      new Waypoint(0, 0, 0),
      new Waypoint(5, 4, Pathfinder.d2r(-45)) 
     };

     //Creates config to be a configured trajectory
    config = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_CUBIC,
      Trajectory.Config.SAMPLES_HIGH, 
      0.05, 
      1.7,//2.8 
      2.0, 
      60.0
    );

    //Creates trajectory using pathfinder to be generated from points and config
    trajectory = Pathfinder.generate(points, config);

    //Creates modifier based on trajectory and base width
    modifier = new TankModifier(trajectory).modify(Robot.drive.baseWidth);

    //Creates variables to handle the encoder potion in relation to the trajectory
    leftEnc = new EncoderFollower(modifier.getLeftTrajectory());
    rightEnc = new EncoderFollower(modifier.getRightTrajectory());

    leftEnc.configureEncoder(Robot.drive.getLeftPositionRaw(), Robot.drive.talonTPR, Robot.drive.wheelDiameter);
    rightEnc.configureEncoder(Robot.drive.getRightPositionRaw(), Robot.drive.talonTPR, Robot.drive.wheelDiameter);

    //Configures encoder given the PID specifications
    leftEnc.configurePIDVA(
      5.0, //kP
      0.0, //kI
      0.0, //kD
      1 / 1.7, //Max Velocity Ratio
      1 //Acceleration Gain
    );
    //Configures encoder given the PID specifications
    rightEnc.configurePIDVA(
      5.0, //kP
      0.0, //kI
      0.0, ///kD
      1 / 1.7, //Max Velocity Ratio
      1 //Acceleration Gain
    );

  }

  @Override
  public void initDefaultCommand() {
  }
  
}
