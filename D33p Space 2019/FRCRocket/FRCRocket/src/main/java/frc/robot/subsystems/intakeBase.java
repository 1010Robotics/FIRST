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

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.teleopIntake;

/**
 * Add your docs here.
 */

 //Creating a public object named "intakeBase" which is a Subsystem with properties for controlling the robot intake
public class intakeBase extends Subsystem {

  private TalonSRX intakeMotor;

  public intakeBase(){

    //Define Motors
    intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR.value);    
    
    //Initialize Intake Motors
    Robot.initTalon(intakeMotor, false);

    //Sets the NeutralMode to Coast
    intakeMotor.setNeutralMode(NeutralMode.Coast);

  }

  public void set(double output){
    intakeMotor.set(ControlMode.PercentOutput, output);
  }

  //Stop Motors
  public void stop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleopIntake());
  }
}
