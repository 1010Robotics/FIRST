/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */
  private TalonFX leftTopM = new TalonFX(1);
  private TalonFX leftBottomM = new TalonFX(0);
  private TalonFX rightTopM = new TalonFX(14);
  private TalonFX rightBottomM = new TalonFX(15);

  public DriveSubsystem() {
    // Make sure they spin forward
    leftTopM.setInverted(false);
    leftBottomM.setInverted(false);
    rightTopM.setInverted(false);
    rightBottomM.setInverted(false);
  }

  public void SetVelocity(double leftVelocity, double rightVelocity) {
    leftBottomM.set(ControlMode.PercentOutput, leftVelocity);  
    leftTopM.set(ControlMode.PercentOutput, leftVelocity);
    rightBottomM.set(ControlMode.PercentOutput, leftVelocity);
    rightTopM.set(ControlMode.PercentOutput, leftVelocity);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
