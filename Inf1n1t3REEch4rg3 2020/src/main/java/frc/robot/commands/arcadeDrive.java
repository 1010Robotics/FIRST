/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.limelight.CameraMode;
import frc.robot.subsystems.limelight.LightMode;

public class arcadeDrive extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final DriveSubsystem chassis;
  private final limelight camera;

  // Exponential Variables
  private final double JoyDead = 0.050;
  private final double DriveExp = 1.7;
  private final double MotorMin = 0.008;

  // Exponential Function
  private double exponential(double joystickVal, double driveExp, double joyDead, double motorMin) {
    double joySign;
    double joyMax = 1 - joyDead;
    double joyLive = Math.abs(joystickVal) - joyDead;
    if (joystickVal > 0) {
      joySign = 1;
    } else if (joystickVal < 0) {
      joySign = -1;
    } else {
      joySign = 0;
    }
    double power = (joySign
        * (motorMin + ((1 - motorMin) * (Math.pow(joyLive, driveExp) / Math.pow(joyMax, driveExp)))));
    if (Double.isNaN(power)) {
      power = 0;
    }
    return power;
  }

  //Joy Code
  private double joyYval;
  private double joyXval;
  private double yOutput;
  private double xOutput;
  //Align Code
  private float moveKp = 0.01f;
  private float moveKd = 0.06f;
  private double moveError;
  private double moveErrorDiff;
  private double moveErrorLast;
	private double moveOutput;

  /**
   * Creates a new arcadeDrive.
   */
  public arcadeDrive(DriveSubsystem sub1, limelight sub2) {
    chassis = sub1;
    camera = sub2;
    addRequirements(chassis);
    addRequirements(camera);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setLedMode(LightMode.eOn);
		camera.setCameraMode(CameraMode.eVision);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //CSVFile.addRow(1, 1, "One", 1);
   // SmartDashboard.putNumber("Left Enc Value", chassis.getLeftPositionRaw());

    if(Robot.oi.main.getXButton()) {
      SmartDashboard.putNumber("CAMERA X", camera.getTx());
      moveError = 0 - camera.getTx();
      moveErrorDiff = moveError - moveErrorLast;
      moveOutput = (moveError * moveKp) + (moveErrorDiff * moveKd);
      chassis.set(ControlMode.PercentOutput, moveOutput, -moveOutput);
      moveErrorLast = moveError;
    }

    else {
      joyYval = Robot.oi.main.getY(Hand.kLeft);
      joyXval = Robot.oi.main.getX(Hand.kRight);
      yOutput = exponential(joyYval, DriveExp, JoyDead, MotorMin);
      xOutput = exponential(joyXval, DriveExp, JoyDead, MotorMin);
      chassis.set(ControlMode.PercentOutput, ((yOutput) - xOutput), ((yOutput) + xOutput));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
