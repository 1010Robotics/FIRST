// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LightMode;
import frc.robot.subsystems.IntakeSubsystem;
import java.lang.Math;

public class AutoSeeking extends CommandBase {
  private final DriveSubsystem chassis;
  private final LimelightSubsystem camera;
  private final IntakeSubsystem intake;
  private double tx;
  private boolean tv;
  private double leftCommand;
  private double rightCommand;
  private double h1=1.5;
  private double h2=6.92;
  private double a1=20;
  private double a2;
  private double getDistance;
  double steeringAdjust = 0.0;

  /** Creates a new AutoSeeking. */
  public AutoSeeking(final DriveSubsystem sub1, final LimelightSubsystem sub4, final IntakeSubsystem sub2) {
    // Use addRequirements() here to declare subsystem dependencies.
    chassis=sub1;
    camera=sub4;
    intake=sub2;
    addRequirements(chassis);
    addRequirements(camera);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    camera.setLedMode(LightMode.eOff);
    tx=camera.getTx();
    tv=camera.isTarget();
    if (tv == false)
    {
        // We don't see the target, seek for the target by spinning in place at a safe speed.
        // 此处记添加若转三圈还找不到目标就后退
        steeringAdjust = 1800;
    }
    else
    {   if(tx<=1&&tx>=-1){
        //the target is within acceptable range
        steeringAdjust = 0;
        }else{
        // We do see the target, execute aiming code
        if(tx>0){
        steeringAdjust =  tx * 30 + 1000;
        }else{
        steeringAdjust = tx * 30 - 1000;
        }
        }

      }


    leftCommand=steeringAdjust;
    rightCommand=-steeringAdjust;
    SmartDashboard.putNumber("left velocity", leftCommand);
    SmartDashboard.putNumber("right velocity", rightCommand);
    SmartDashboard.putNumber("adjustValue", steeringAdjust);
    chassis.set(ControlMode.Velocity, rightCommand, leftCommand);
    //start driving towards the target
    a2 = camera.getTy();
    getDistance = (h2-h1) / Math.tan((a1+a2)*Math.PI/180) - 1;
    SmartDashboard.putNumber("current distance is ",getDistance);
    SmartDashboard.putNumber("a2 ",a2);
    
    //if current distance from the target is bigger than this value
    if(getDistance>=10){//ft
    chassis.set(ControlMode.Velocity, -2000, -2000);
    }else{
    chassis.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}