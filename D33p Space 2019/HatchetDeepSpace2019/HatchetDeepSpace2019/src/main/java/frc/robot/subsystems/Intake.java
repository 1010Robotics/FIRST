/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeCommand;

public class Intake extends Subsystem {

  public final int talonTPR = 4069; //Full Encoder Rotation
  public final int INTAKEFLOORPRESET = 500;//TUNE
  public final int INTAKEUPPRESET = 0; //TUNE 
  public final int OUTOFWAYPRESET = 250;//TUNE
  public final int INWAYRANGETOP = 249;//TUNE
  public final int INWAYRANGEBOTTOM = 251;//TUNE

    //Motors
  private TalonSRX intakeMotor;

    //Variables
  private float P = 0.0f;//Fine TUNE
  private float I = 0.0f;//Fine TUNE
  private float D = 0.0f;//Fine TUNE
  private double derivative, error, integral = 0;


  public Intake() {
    intakeMotor = new TalonSRX(RobotMap.INTAKE.getValue());// defining intake talonr

    Robot.initTalon(intakeMotor, false);

    Robot.initMasterDriveMotor(intakeMotor);
}

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeCommand());
  }
  
  public void executePIDToPosition(double desiredPosition) 
  {
      
      while (getIntakePosition() != desiredPosition) {
          error = desiredPosition - getIntakePosition(); // Error = Target - Actual
          this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
          derivative = (error - this.error) / .02;
          double intakeValue = P*error + I*this.integral + D*derivative;
          intakeValue = (intakeValue > 0.25 ? 0.25: intakeValue < -0.25 ? -0.25 : intakeValue); //TUNE NOTE: motor speed
          intakeMotor.set(ControlMode.PercentOutput, intakeValue);
      }
  }

  //Get Intake Encoder
  public double getIntakePosition() { //getIntakePosition() represents the current value of intakeMotor
    return intakeMotor.getSensorCollection().getQuadraturePosition();
  }

  public void moveIntakeToFloorPreset() {
    executePIDToPosition(INTAKEFLOORPRESET);
  }

  public void moveIntakeToUpPreset() {
    executePIDToPosition(INTAKEUPPRESET);
  }

  public void moveIntakeOutOfWay() {

    if (getIntakePosition() < INWAYRANGETOP && getIntakePosition() > INWAYRANGEBOTTOM) {
      executePIDToPosition(OUTOFWAYPRESET);
    }
  
  }

}