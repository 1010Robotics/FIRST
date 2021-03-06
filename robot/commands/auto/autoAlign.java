/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.subsystems.limeLight.CameraMode;
import frc.robot.subsystems.limeLight.LightMode;

public class autoAlign extends Command {

  //PID Constants
  private float headingKp = 0.014f;
  private float moveKp = 0.039f;

  //PID Variables
  private double headingError;
  private double moveError;
  private double headingOutput;
  private double moveOutput;

  public autoAlign() {
    requires(Robot.drive);
    requires(Robot.camera);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {

    Robot.camera.setLedMode(LightMode.eOn);
    Robot.camera.setCameraMode(CameraMode.eVision);

    if(Robot.camera.isTarget() == true) {
      moveError = -12 - Robot.camera.getTy();
      headingError = 1.37 - Robot.camera.getTx();
      headingOutput = headingError * headingKp;
      moveOutput = moveError * moveKp;

      Robot.drive.set(ControlMode.PercentOutput, moveOutput-headingOutput, moveOutput+headingOutput);
    }
    else {
      Robot.drive.set(ControlMode.PercentOutput, -0.3, 0.3);
    }

    try { TimeUnit.MILLISECONDS.sleep(10); } 	
    catch (Exception e) { /* Do Nothing */ } 
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
