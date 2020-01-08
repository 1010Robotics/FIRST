/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveBase extends SubsystemBase {

  //Declare Motors
  public CANSparkMax baseLeft, baseRight;
  public CANEncoder leftEncoder, rightEncoder;
  
  public static void initSparkMax(final CANSparkMax Spark, CANEncoder Encoder, boolean inverted) {
    //Restores defaults
    Spark.restoreFactoryDefaults();
    //Declares Encoder
    Encoder = Spark.getEncoder();
    Encoder.setPosition(0);

    //Sets Ramp Rate and Mode
    Spark.setOpenLoopRampRate(0);
    Spark.setIdleMode(IdleMode.kBrake);

    //Invert our right side motor
    Spark.setInverted(inverted);
  }

  public driveBase() {
    // Define Motors
    try{baseLeft = new CANSparkMax(Constants.RobotMap.LEFT_MOTOR.value, MotorType.kBrushless);}
    catch (RuntimeException ex){DriverStation.reportError("Error Starting CANSParkMax: " + ex.getMessage(), true);}
    try{baseRight = new CANSparkMax(Constants.RobotMap.RIGHT_MOTOR.value, MotorType.kBrushless);}
    catch (RuntimeException ex){DriverStation.reportError("Error Starting CANSParkMax: " + ex.getMessage(), true);}
    // Initialize Motors
    initSparkMax(baseLeft, leftEncoder, false);
    initSparkMax(baseRight, rightEncoder, true);
  }

  public double getLeftPositionRaw() {
    return leftEncoder.getPosition();
  }

  public double getRightPositionRaw(){
    return rightEncoder.getPosition();
  }
  
  public void resetEnc(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public void set(double rightVal, double leftVal){ //input between -1.0 and 1.0
    baseLeft.set(leftVal);
    baseRight.set(rightVal);
  }

  public void stop(){
    baseLeft.set(0);
    baseRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
