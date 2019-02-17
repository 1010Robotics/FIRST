/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Imports
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.teleopIntake;

/**
 * Add your docs here.
 */

 //Creating a public object named "intakeBase" which is a Subsystem with properties for controlling the robot intake
public class intakeBase extends Subsystem {

  //Declares intakeMotor as a TalonSRX motor
  private TalonSRX intakeMotor;

  //Creates public intake function
  public intakeBase(){
    //Declares intakeMotor as a TalonSRX motor
    intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR.value);    
    
    //Initialize intake motors
    Robot.initTalon(intakeMotor, false);

    //Sets the neutral mode to coast
    intakeMotor.setNeutralMode(NeutralMode.Coast);
  }

  //Sets percent output for motors
  public void set(double output){
    intakeMotor.set(ControlMode.PercentOutput, output);
  }

  //Stops motor by setting percentage output to 0%
  public void stop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void initDefaultCommand() {
    //Set the default command for a subsystem here.
    setDefaultCommand(new teleopIntake());
  }
}
