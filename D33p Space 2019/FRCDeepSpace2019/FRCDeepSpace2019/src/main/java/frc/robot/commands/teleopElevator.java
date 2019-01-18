/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.elevatorBase.elevatorPosition;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class teleopElevator extends Command {

  //Variables
  int currentHeight;
  private ShuffleboardLayout elevatorPresetButtons;
  private NetworkTableEntry elevatorMotorControl;
  public ShuffleboardTab testTab = Shuffleboard.getTab("Test Tab");
  //Controller Buttons
  boolean joyA = Robot.oi.main.getAButton();
  boolean joyB = Robot.oi.main.getBButton();
  boolean joyX = Robot.oi.main.getXButton();

  public teleopElevator() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    testTab = Shuffleboard.getTab("Test Tab");
    
    elevatorMotorControl = testTab
    .add("Elevator Motor", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("MIN", -1, "MAX", 1))
    .getEntry();

    elevatorPresetButtons = testTab
    .getLayout("Current Preset", BuiltInLayouts.kList);
    elevatorPresetButtons.add("test_1", false);
    elevatorPresetButtons.add("test_2", true);
  }

  @Override
  protected void execute() {


    if(joyA==true){
      currentHeight = Robot.elevator.LOW_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.LOW;
    }
    else if(joyB==true){
      currentHeight = Robot.elevator.MID_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.MID;
    }
    else if(joyX==true){
      currentHeight = Robot.elevator.HIGH_GOAL;
      Robot.elevator.elevatorState = elevatorPosition.HIGH;
    }
    //SmartDashboard.putString("Elevator State", Robot.elevator.elevatorState.toString());
    if(DriverStation.getInstance().isTest()){
      Robot.elevator.set(ControlMode.PercentOutput, elevatorMotorControl.getDouble(0.00));
    }
    //Robot.elevator.set(ControlMode.MotionMagic, currentHeight);
    
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.elevator.stop();
  }

  @Override
  protected void interrupted() {
  }
}
