/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
  * @param mode CTRE TalonFX ControlMode
  * @param rightVal Right Drive Motors' Output
  * @param leftVal Left Drive Motors' Output
  */
  public void set(ControlMode mode, double rightVal, double leftVal) {
    leftMaster.set(mode, leftVal);
    rightMaster.set(mode, rightVal);
  }

  /*
  */
  public void stop(){
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);
  }

  /*
  */
  public double getLeftPositionRaw() {
    return leftMaster.getSelectedSensorPosition();
  }

  /*
  */
  public double getRightPositionRaw(){
    return rightMaster.getSelectedSensorPosition();
  }
  
  /*
  */
  public void resetEnc() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /*
  */
  public void getAngle() {
    gyro.getAngle();
  }

  /*
  */
  public void resetAngle() {
    gyro.reset();
  }

  /*
  */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
