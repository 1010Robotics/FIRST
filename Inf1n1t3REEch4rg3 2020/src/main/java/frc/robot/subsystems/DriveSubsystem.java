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
 * - Odometry Calculations and Methods
 * - PathFollowing Code
 * - Sensors Value Methods
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.InitializeTalon;

public class DriveSubsystem extends SubsystemBase {

  //Declare Motors
  private TalonFX leftMaster;
  private TalonFX leftSlave;
  private TalonFX rightMaster;
  private TalonFX rightSlave;
  //Declare Sensors
  private AHRS gyro;
  //Declare Others
  private DifferentialDriveOdometry mOdometry;

  public DriveSubsystem() {
    //Define Motors
    try {
      leftMaster = new TalonFX(Constants.RobotMap.LEFT_MASTER.value);
      leftSlave = new TalonFX(Constants.RobotMap.LEFT_SLAVE.value);
      rightMaster = new TalonFX(Constants.RobotMap.RIGHT_MASTER.value);
      rightSlave = new TalonFX(Constants.RobotMap.RIGHT_SLAVE.value);
    }
    catch (RuntimeException ex) {DriverStation.reportError("Error Starting TalonFX: " + ex.getMessage(), true);}

    //Initialize Motors
    InitializeTalon.initLeftDriveFalcon(leftMaster);
    InitializeTalon.initLeftDriveFalcon(leftSlave);
    InitializeTalon.initRightDriveFalcon(rightMaster);
    InitializeTalon.initRightDriveFalcon(rightSlave);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);


    //Set Slaves
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    //Define Gyro
    try {gyro = new AHRS(SPI.Port.kMXP);}
    catch (RuntimeException ex) {DriverStation.reportError("Error Starting NavX: " + ex.getMessage(), true);}

    resetEnc();
    resetAngle();

    mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
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
   * Get Left Wheels' Position in Meters
   * 
   * @return Current Position for the Left Drive, in meters
   */
  public double getLeftPosition() {
    return leftMaster.getSelectedSensorPosition(); //Code to Meters
  }

  /**
   * Get Left Wheels' Velocity in Meters per Second
   * 
   * @return Current Velocity of the Left Drive, in meters per second
   */
  public double getLeftVelocity() {
    return leftMaster.getSelectedSensorVelocity(); //Code to Meters per Second
  }

  /**
   * Get Right Wheels' Position in Raw Encoder Units (2048 per Revolution)
   * 
   * @return Current Raw Encoder Units for the Right Drive
   */
  public double getRightPositionRaw(){
    return rightMaster.getSelectedSensorPosition();
  }

  /**
   * Get Right Wheels' Position in Meters
   * 
   * @return Current Position for the Right Drive, in meters
   */
  public double getRightPosition() {
    return rightMaster.getSelectedSensorPosition(); //Code to Meters
  }

  /**
   * Get Right Wheels' Velocity in Meters per Second
   * 
   * @return Current Velocity of the Right Drive, in meters per second
   */
  public double getRightVelocity() {
    return rightMaster.getSelectedSensorVelocity(); //Code to Meters per Second
  }
  
  /**
   * Reset all Drive Encoders to zero
   */
  public void resetEnc() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /**
   * Get Current NavX's Yaw Angle
   * 
   * @return Robot's Current Angle
   */
  public double getAngle() {
    return gyro.getYaw();
  }

  /**
   * Get Current Rate of Change in Angle of NavX's Yaw
   * 
   * @return Robot's Current Rate of Change in Angle
   */
  public double getAngleRate() {
    return gyro.getRate();
  }

  /**
   * Reset NavX's Yaw Angle to zero
   */
  public void resetAngle() {
    gyro.reset();
  }

  /**
   * Returns the currently-estimated pose of the robot
   *
   * @return The pose
   */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose
   *
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEnc();
    mOdometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    mOdometry.update(Rotation2d.fromDegrees(getAngle()), getLeftPosition(), getRightPosition());
  }
}
