/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.teleopIntake;

/**
 * Add your docs here.
 */
public class intakeBase extends SubsystemBase {

  private TalonSRX intakeMotor;

  public intakeBase(){
    //Define Motors
    intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR.value);

    //Initialize Intake Motors
    Robot.initTalon(intakeMotor, true);

    //Sets the NeutralMode to Coast
    intakeMotor.setNeutralMode(NeutralMode.Coast);
  }

  //Set Motors
  public void set(double output){
    intakeMotor.set(ControlMode.PercentOutput, output);
  }

  //Stop Motors
  public void stop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  //Get Percent Output in a Percentage (-1 to 1)
  public double getIntakeOutput(){
    return intakeMotor.getMotorOutputPercent();
  }

/*   @Override
  public void initDefaultCommand() {
    setDefaultCommand(new teleopIntake());
  } */
}
