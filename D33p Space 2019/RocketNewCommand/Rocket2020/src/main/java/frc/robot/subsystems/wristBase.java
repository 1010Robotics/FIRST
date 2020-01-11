/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class wristBase extends SubsystemBase {

  public enum wristPosition{INTAKE_WALL, INTAKE_FLOOR, CARGO_SCORE} //you get the idea
  
  //sets default for the enum
  public wristPosition wristState = wristPosition.INTAKE_FLOOR; 

  private TalonSRX wristMotor;

  public int INTAKE_HATCH = 3100; //preset for intake (and default position)
  public int INTAKE_BALL = 3700; //preset for cargo
  public int SCORE_BALL = 0;
  public int SCORE_HATCH = 0;//450; //preset for hatch

  public wristBase() {
    //Define wrist motor
    wristMotor = new TalonSRX(Constants.RobotMap.WRIST_MOTOR.value);

    //Initialize Wrist Motor
    Robot.initTalon(wristMotor, false);
    
    //Setting Closed Control Loop and MotionMagic, just like the Elevator
    Robot.initMasterWristMotor(wristMotor);
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
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
