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
import frc.robot.commands.teleopWrist;

/**
 * Add your docs here.
 */
public class wristBase extends Subsystem {
  public enum wristPosition{INTAKE_WALL, INTAKE_FLOOR} //you get the idea
  public wristPosition wristState = wristPosition.INTAKE_WALL;

  private TalonSRX wristMotor;

  public int LOW_HATCH = 0;
  public int POS_2 = 12000; //once again, you get the idea

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public wristBase(){
    //Define wrist motor
    wristMotor = new TalonSRX(RobotMap.WRIST_MOTOR.value);

    //Initialize Wrist Motor
    Robot.initTalon(wristMotor, false);

    //Setting Closed Control Loop and MotionMagic, just like the Elevator
    Robot.initMasterElevatorMotor(wristMotor);
  }

 
  //Set Motors
  public void set(ControlMode mode, double output) {
    wristMotor.set(mode, output);
  }

  //Stop Motors
  public void stop() {
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

  
  //Get Current Position in Encoder Units
  public double getWristPosition(){
    return wristMotor.getSensorCollection().getQuadraturePosition();
  }

   //Get Current Output in Percent (-1 to 1)
  public double getWristOutput() {
    return wristMotor.getMotorOutputPercent();
  }

  //Reset Encoder on the Wrist
  public void resetEnc() {
    wristMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleopWrist());
  }
}
