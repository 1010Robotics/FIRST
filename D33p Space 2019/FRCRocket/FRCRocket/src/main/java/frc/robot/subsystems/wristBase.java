/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.teleopWrist;

/**
 * Add your docs here.
 */

//Creating a public object named "wristBase" which is a Subsystem with properties for controlling the robot wrist
public class wristBase extends Subsystem {
  public enum wristPosition{INTAKE_WALL, INTAKE_FLOOR, CARGO_SCORE} //you get the idea
  public wristPosition wristState = wristPosition.INTAKE_FLOOR;

  //Declares wristMotor as a TalonSRX motor
  private TalonSRX wristMotor;

  //Creates public integers
  public int INTAKE_POS = 400; 
  public int CARGO_POS = 3200; //once again, you get the idea
  public int HATCH_POS = 4600; //once again, you get the idea

  //Creates public function "wristBase"
  public wristBase(){
    //Define wrist motor
    wristMotor = new TalonSRX(RobotMap.WRIST_MOTOR.value);

    //Initialize wrist motor
    Robot.initTalon(wristMotor, false);

    //Setting closed control loop and motion magic, just like the Elevator
    Robot.initMasterWristMotor(wristMotor);
  }

  //Set motors settigns (mode and output %)
  public void set(ControlMode mode, double output) {
    wristMotor.set(mode, output);
  }

  //Set motor output to 0% to stop motor
  public void stop() {
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

  //Get current position in encoder units
  public double getWristPosition(){
    return wristMotor.getSensorCollection().getQuadraturePosition();
  }

   //Get current output in percent (-1 to 1)
  public double getWristOutput() {
    return wristMotor.getMotorOutputPercent();
  }

  //Reset encoder on the wrist
  public void resetEnc() {
    wristMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleopWrist());
  }
}
