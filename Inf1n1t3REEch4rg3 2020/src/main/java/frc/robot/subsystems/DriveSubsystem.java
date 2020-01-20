/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Pedro Amui and Caden Hewlett
 * 
 * DriveSubsystem for the 6364 Infinite Recharge Robot.
 * Includes Left and Right Side Motors of the Drivetrain.
 * - 4 Falcon500 Motors (2 on each side)
 * - 4 Built-in Encoders (1 per motor)
 * - 1 NavX Gyro on RoboRio SPI Port
 * 
 * Key Methods and Utilities:
 * - PathFollowing Code
 * - Sensors Value Methods
 * - Anti-tipping Code
 */


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

  //Declare Motors
  TalonFX leftMaster;
  TalonFX leftSlave;
  TalonFX rightMaster;
  TalonFX rightSlave;
  //Declare Sensors
  AHRS gyro;

  public DriveSubsystem() {
    //Define Motors
    try {
      leftMaster = new TalonFX(0);
      leftSlave = new TalonFX(1);
      rightMaster = new TalonFX(2);
      rightSlave = new TalonFX(3);
    }
    catch (RuntimeException ex) {DriverStation.reportError("Error Starting TalonFX: " + ex.getMessage(), true);}

    //Initialize Motors
    Robot.initFalcon(leftMaster);
    Robot.initFalcon(leftSlave);
    Robot.initFalcon(rightMaster);
    Robot.initFalcon(rightSlave);

    //Set Slaves
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    //Define Gyro
    try {gyro = new AHRS(SPI.Port.kMXP);}
    catch (RuntimeException ex) {DriverStation.reportError("Error Starting NavX: " + ex.getMessage(), true);}
  }

  /**
   * Set Drive Motors to certain Percent Output
   * 
   * @param rightVal Right Drive Motors' Percent Output
   * @param leftVal Left Drive Motors' Percent Output
   */
  public void set(double rightVal, double leftVal) {
    leftMaster.set(ControlMode.PercentOutput, leftVal);
    rightMaster.set(ControlMode.PercentOutput, rightVal);
  }

  /**
   * Set Drive Motors to certain Control Mode and Output
   * 
   * @param mode CTRE TalonFX ControlMode
   * @param rightVal Right Drive Motors' Output
   * @param leftVal Left Drive Motors' Output
   */
  public void set(ControlMode mode, double rightVal, double leftVal) {
    leftMaster.set(mode, leftVal);
    rightMaster.set(mode, rightVal);
  }

  /**
   * Stop Drive Motors
   */
  public void stop(){
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Get Left Position in Raw Encoder Units (2048 per Revolution)
   * 
   * @return Current Raw Encoder Units for the Left Drive
   */
  public double getLeftPositionRaw() {
    return leftMaster.getSelectedSensorPosition();
  }

  /**
   * Get Right Position in Raw Encoder Units (2048 per Revolution)
   * 
   * @return Current Raw Encoder Units for the Right Drive
   */
  public double getRightPositionRaw(){
    return rightMaster.getSelectedSensorPosition();
  }
  
  /**
   * Reset all Drive Encoders to zero
   */
  public void resetEnc() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /**
   * Get Current NavX X-Axis Angle
   * 
   * @return Return Robot's Current Angle
   */
  public double getAngle() {
    return gyro.getAngle();
  }

  /**
   * Reset NavX's Z-Axis Angle
   */
  public void resetAngle() {
    gyro.reset();
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
  }
}
