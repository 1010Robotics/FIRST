/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.driveBase;

public class arcadeDrive extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final driveBase chassis;
  
  //Exponential Variables
	private final double JoyDead = 0.050;//was 0.05????????????????????????????????????????????????????????? JOY DEAD
	private final double DriveExp = 1.7;//was 1.9!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! EXPO
	private final double MotorMin = 0.008;
  	//Exponential Function
	private double exponential(double joystickVal, double driveExp, double joyDead, double motorMin){
		double joySign;
		double joyMax = 1 - joyDead;
		double joyLive = Math.abs(joystickVal) - joyDead;
		if (joystickVal > 0) {joySign = 1;}
		else if (joystickVal < 0) {joySign = -1;}
		else {joySign = 0;}
		double power = (joySign * (motorMin + ((1 - motorMin) * (Math.pow(joyLive, driveExp) / Math.pow(joyMax, driveExp)))));
		if(Double.isNaN(power)){power = 0;}
		return power;
  }
  
	private double joyYval;
	private double joyXval;
	private double yOutput;
  private double xOutput;

  /**
   * Creates a new arcadeDrive.
   */
  public arcadeDrive(driveBase subsystem) {
    chassis = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Left Enc Value", chassis.getLeftPositionRaw());

    joyYval = Robot.oi.main.getY(Hand.kLeft);
		joyXval = Robot.oi.main.getX(Hand.kRight);

		yOutput = exponential(joyYval, DriveExp, JoyDead, MotorMin);
		xOutput = exponential(joyXval, DriveExp, JoyDead, MotorMin);

    //SmartDashboard.putNumber("Joystick?", joyXval);

    chassis.set(-((yOutput) + xOutput), -((yOutput) - xOutput));
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
