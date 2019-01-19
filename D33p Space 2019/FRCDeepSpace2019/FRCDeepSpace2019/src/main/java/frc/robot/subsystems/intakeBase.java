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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.teleopIntake;

/**
 * Add your docs here.
 */
public class intakeBase extends Subsystem {

  private TalonSRX intakeMotor;
  private VictorSPX intakeMotorF;

  public intakeBase(){
    //Define Motors
    intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR.value);
    intakeMotorF = new VictorSPX(RobotMap.INTAKE_MOTORF.value);

    //Initialize Intake Motors
    Robot.initTalon(intakeMotor, false);
    Robot.initVictor(intakeMotorF, false);

    //Sets the NeutralMode to Coast
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    //Enslave Victor to Talon
    intakeMotorF.follow(intakeMotor);
  }

  //Set Motors
  public void set(ControlMode mode, double output){
    intakeMotor.set(mode, output);
  }

  //Stop Motors
  public void stop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  //Get Percent Output in a Percentage (-1 to 1)
  public double getIntakeOutput(){
    return intakeMotor.getMotorOutputPercent();
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleopIntake());
  }
}
