/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorCommand;

public class Elevator extends Subsystem {

    public final double ELEVATORUPPRESET = 1000;// {1000,2000} //TUNE
    public final double ELEVATORDOWNPRESET = 0.0;//TUNE

      //Motors
    private TalonSRX elevatorMotor;
    private VictorSPX elevatorMotorF;   

    
      //Variables
    private float P = 0.0f; //NOTE: TUNE (around 0.0001)
    private float I = 0.0f;//TUNE
    private float D = 0.0f; //TUNE
    private double derivative, error, integral = 0;
    

    public final int talonTPR = 4096; //Full Encoder Rotation



    public Elevator() {
        elevatorMotor = new TalonSRX(RobotMap.ELEVATOR.getValue());// defining elevator talon
        elevatorMotorF = new VictorSPX(RobotMap.ELEVATORF.getValue());// defining elevator victor

        Robot.initTalon(elevatorMotor, false);
        Robot.initVictor(elevatorMotorF, false);

        elevatorMotorF.follow(elevatorMotor);

        Robot.initMasterDriveMotor(elevatorMotor);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ElevatorCommand());
    }

    public void executePIDToPosition(double desiredPosition)
    {
        
        while (getElevatorPosition() != desiredPosition) {
            error = desiredPosition - getElevatorPosition(); // Error = Target - Actual
            this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
            derivative = (error - this.error) / .02;
            double elevatorValue = P*error + I*this.integral + D*derivative;
            elevatorValue = (elevatorValue > 0.25 ? 0.25 : elevatorValue < -0.25 ? -0.25 : elevatorValue); // TUNE NOTE: motor speed
            elevatorMotor.set(ControlMode.PercentOutput, elevatorValue);
        }
    }

    //Get Elevator Encoder in Feet
    public double getElevatorPosition() {
        //return -((elevatorMotor.getSensorCollection().getPulseWidthPosition()/talonTPR)*1.57);
        return elevatorMotor.getSensorCollection().getQuadraturePosition();
    }  
        
    public void moveElevatorToUpPreset() {
        executePIDToPosition(ELEVATORUPPRESET);
    }

    public void moveElevatorToDownPreset() {
        executePIDToPosition(ELEVATORDOWNPRESET);
    }


}