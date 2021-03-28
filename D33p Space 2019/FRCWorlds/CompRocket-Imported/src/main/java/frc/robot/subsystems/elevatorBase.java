/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.teleopElevator;

public class elevatorBase extends SubsystemBase {

  //Variables 
  public enum elevatorPosition{LOW, SCORE_LOW, MID, SCORE_MID, HIGH, SCORE_HIGH, BALL_LOAD} 
  public elevatorPosition elevatorState = elevatorPosition.LOW;

  //Preset Heights
  public double currentHeight;
  public int LOW_GOAL = 0;
  public int MID_GOAL = 16000;
  public int HIGH_GOAL = 26000;
  public int LOW_GOAL_FRONT = 5000;
  public int MID_GOAL_FRONT = 16000;
  public int HIGH_GOAL_FRONT = 26000; //was 24750
  public int BALL_LOAD = 11000;

  //500 2 inches

  //Motors
  private TalonSRX encMotor;
  private TalonSRX encMotorF;
  
  public elevatorBase() {

    //Define Motors
    encMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR.value);
    encMotorF = new TalonSRX(RobotMap.ELEVATOR_MOTORF.value);

    //Initialize Motors
    Robot.initTalon(encMotor, false);
    Robot.initTalon(encMotorF, false);

    //Set Closed Control Loop and Motion Magic Configuration
    Robot.initMasterElevatorMotor(encMotor);
    encMotorF.follow(encMotor);
  }

  //Set Motors
  public void set(ControlMode mode, double output) {
    encMotor.set(mode, output);
  }

  //Stop Motors
  public void stop() {
    encMotor.set(ControlMode.PercentOutput, 0);
  }

  //Get Current Position in Encoder Units
  public double getElevatorPosition() {
    return encMotor.getSensorCollection().getQuadraturePosition();
  }

  //Get Current Output in Percent (-1 to 1)
  public double getElevatorOutput() {
    return encMotor.getMotorOutputPercent();
  }

  //Reset Encoder on the Elevator
  public void resetEnc() {
    encMotor.setSelectedSensorPosition(0);
  }

  //Default Command
 /* @Override
  public void initDefaultCommand() {
    setDefaultCommand(new teleopElevator());
  }*/
}

