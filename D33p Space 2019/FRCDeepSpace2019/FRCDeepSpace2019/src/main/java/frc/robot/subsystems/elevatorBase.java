/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.teleopElevator;

public class elevatorBase extends Subsystem {

  //Constants 
  public enum elevatorPosition{LOW, MID, HIGH} 
  public elevatorPosition elevatorState = elevatorPosition.LOW;

  //Preset Heights
  public int LOW_GOAL = 0;
  public int MID_GOAL = 10000;
  public int HIGH_GOAL = 20000;

  //Hardware
  private TalonSRX encMotor;

  public elevatorBase() {

    //Create Objects
    encMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR.value); //declare new Talon

    //Initialize Motors
    Robot.initTalon(encMotor, true); //initialize a new Talon
    Robot.initMasterElevatorMotor(encMotor); //see Robot.java
  }

  public void set(ControlMode mode, double output) {
    encMotor.set(mode, output);
  }

  public void stop() {
    encMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getElevatorPosition() {
    return encMotor.getSensorCollection().getQuadraturePosition();
  }

  public double getElevatorOutput() {
    return encMotor.getMotorOutputPercent();
  }

  public void resetEnc() {
    encMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new teleopElevator());
  }
}
