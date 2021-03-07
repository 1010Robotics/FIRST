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
  private double left_command;
  private double right_command;
  double steering_adjust = 0.0;
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
        steering_adjust = 0;
    }
    else
    {   if(tx<=2&&tx>=-2){
        //the target is within acceptable range
        steering_adjust = 1200;
        }else{
        // We do see the target, execute aiming code
        // kp*tx, this value should be around 2 digits, 10 is used to prevent the value being too small;
        steering_adjust =  tx * 20 + 875;
        }
    }

    left_command=steering_adjust;
    right_command=-steering_adjust;
    SmartDashboard.putNumber("left velocity", left_command);
    SmartDashboard.putNumber("right velocity", right_command);
    SmartDashboard.putNumber("adjustValue", steering_adjust);
    chassis.set(ControlMode.Velocity, right_command, left_command);
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
