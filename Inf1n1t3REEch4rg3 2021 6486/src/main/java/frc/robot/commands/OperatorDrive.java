/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.CameraMode;
import frc.robot.subsystems.LimelightSubsystem.LightMode;
import frc.robot.utilities.Exponential;

public class OperatorDrive extends CommandBase {

  private final DriveSubsystem chassis;
  private final LimelightSubsystem camera;

  // Exponential Variables
  private final double JoyDead = 0.050;
  private final double DriveExp = 1.7;
  private final double MotorMin = 0.008;

  // Joy Code
  private double joyYvalL;
  private double joyYvalR;
  private double yOutputL;
  private double yOutputR;
  private double cOutput;

  // Align Code
  private final float moveKp = 0.01f;
  private final float moveKd = 0.06f;
  private double moveError;
  private double moveErrorDiff;
  private double moveErrorLast;
  private double moveOutput;

  /**
   * Creates a new arcadeDrive.
   */
  public OperatorDrive(final DriveSubsystem sub1, final LimelightSubsystem sub2) {
    chassis = sub1;
    camera = sub2;
    addRequirements(chassis);
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setLedMode(LightMode.eOff);
    camera.setCameraMode(CameraMode.eVision);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /**
     * SMARTDASHBOARD
     */

    SmartDashboard.putNumber("Right Drive Velocity Raw", chassis.getRightVelocity());
    SmartDashboard.putNumber("Left Drive Velocity Raw", chassis.getLeftVelocity());
    SmartDashboard.putNumber("GyroVal", chassis.getAngle());
    SmartDashboard.putNumber("Correction Output", cOutput);
    SmartDashboard.putNumber("Limelight X-Axis", camera.getTx());

    /**
     * AIMER
     */

    if (Robot.oi.main.getBumper(Hand.kLeft)) {
      camera.setLedMode(LightMode.eOn);

      moveError = 0 - camera.getTx();
      moveErrorDiff = moveError - moveErrorLast;
      moveOutput = (moveError * moveKp) + (moveErrorDiff * moveKd);

      chassis.set(ControlMode.PercentOutput, moveOutput, -moveOutput);

      moveErrorLast = moveError;
    }

    /**
     * DRIVE
     */

    else {
      camera.setLedMode(LightMode.eOff);

      joyYvalL = Robot.oi.main.getY(Hand.kLeft);
      joyYvalR = Robot.oi.main.getY(Hand.kRight);
      yOutputL= 21000 * Exponential.exponential(joyYvalL, DriveExp, JoyDead, MotorMin);
      yOutputR = 21000 * Exponential.exponential(joyYvalR, DriveExp, JoyDead, MotorMin);

      chassis.set(ControlMode.Velocity,  -(yOutputR),-(yOutputL));
    }
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
