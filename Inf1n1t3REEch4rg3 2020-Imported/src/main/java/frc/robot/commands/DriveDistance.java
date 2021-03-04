/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utilities.PID;
import java.lang.Math;

import com.ctre.phoenix.motorcontrol.ControlMode;


public class DriveDistance extends CommandBase {

  private PID drivePID;
  private final DriveSubsystem chassis;
  private final LimelightSubsystem camera;
  private int distance;
  private int direction;
  private double getDistance;
  private boolean isTarget;

  private double h1=1;
  private double h2=7;
  private double a1=20;
  private double a2;

  public DriveDistance(int direction, int distance, final DriveSubsystem sub1, final LimelightSubsystem sub4) {
    chassis = sub1;
    camera = sub4;
    addRequirements(chassis);
    addRequirements(camera);
    direction = this.direction;
    distance = this.distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID.set_PID_vars(0, 0, 0, 0);
    drivePID.target = distance * direction;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivePID.current = (int) chassis.getLeftPosition();
    isTarget = camera.isTarget();
    a2 = camera.getTy();
    getDistance = (h2-h1) / Math.tan(a1+a2);
    SmartDashboard.putNumber("current distance is ",getDistance);
    if(isTarget){
      //if current distance from the target is bigger than this value
    if(getDistance>=6){
        //chassis.set(drivePID.output(0.6), drivePID.output(0.6));
        //if(Math.abs(drivePID.current) >= Math.abs(drivePID.target)){
          //chassis.stop();
          //isFinished();
        //}
      //}else{
        //chassis.stop();
      //}
      chassis.set(ControlMode.Velocity, 40, 40);
    }else{
      chassis.stop();
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
