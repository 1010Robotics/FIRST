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

public class AutoSeeking extends CommandBase {
  private final DriveSubsystem chassis;
  private final LimelightSubsystem camera;
  private double tx;
  private boolean tv;
  private double leftCommand;
  private double rightCommand;
  double steeringAdjust = 0.0;

  /** Creates a new AutoSeeking. */
  public AutoSeeking(final DriveSubsystem sub1, final LimelightSubsystem sub4) {
    // Use addRequirements() here to declare subsystem dependencies.
    chassis=sub1;
    camera=sub4;
    addRequirements(chassis);
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    camera.setLedMode(LightMode.eOn);
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
