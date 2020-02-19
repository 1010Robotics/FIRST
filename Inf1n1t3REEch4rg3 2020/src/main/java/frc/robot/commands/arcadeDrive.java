/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.concurrent.TimeUnit;

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


public class arcadeDrive extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final DriveSubsystem chassis;
  private final LimelightSubsystem camera;

  // Exponential Variables
  private final double JoyDead = 0.050;
  private final double DriveExp = 1.7;
  private final double MotorMin = 0.008;

  // Joy Code
  private double joyYval;
  private double joyXval;
  private double yOutput;
  private double xOutput;
  private double cOutput;
  //private double kSkew = 4500; //should be in constants.java

  // Align Code
  private boolean correcting = false;
  private int count = 0;
  private final float moveKp = 0.01f;
  private final float moveKd = 0.06f;
  private double startingAngle;
  private double angleError;
  private double angleErrorDiff;
  private double angleErrorLast;
  private double moveError;
  private double moveErrorDiff;
  private double moveErrorLast;
  private double moveOutput;

  /**
   * Creates a new arcadeDrive.
   */
  public arcadeDrive(final DriveSubsystem sub1, final LimelightSubsystem sub2) {
    chassis = sub1;
    camera = sub2;
    addRequirements(chassis);
    addRequirements(camera);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setLedMode(LightMode.eOn);
    camera.setCameraMode(CameraMode.eVision);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Right Drive Velocity Raw", chassis.getRightVelocity());
    SmartDashboard.putNumber("Left Drive Velocity Raw", chassis.getLeftVelocity());
    SmartDashboard.putNumber("GyroVal", chassis.getAngle());
    SmartDashboard.putNumber("Angle Rate", chassis.getAngleRate());
    SmartDashboard.putNumber("Correction Output", cOutput);

    if (Robot.oi.main.getXButton()) {
      SmartDashboard.putNumber("CAMERA X", camera.getTx());
      moveError = 0 - camera.getTx();
      moveErrorDiff = moveError - moveErrorLast;
      moveOutput = (moveError * moveKp) + (moveErrorDiff * moveKd);
      chassis.set(ControlMode.PercentOutput, moveOutput, -moveOutput);
      moveErrorLast = moveError;
    }

    else {

      correcting = (Math.abs(Robot.oi.main.getY(Hand.kLeft)) > JoyDead) && 
        (Math.abs(Robot.oi.main.getX(Hand.kRight)) < JoyDead);

      if(!correcting){count = 0;}

      if(correcting) { 

        if(count == 0){
          count = 1;
          chassis.resetAngle();
          startingAngle = chassis.getAngle();
          try{ TimeUnit.MILLISECONDS.sleep(150);}
            catch(Exception ex){/*DELAY*/}
        }

        angleError = startingAngle - chassis.getAngle();
        //angleErrorDiff = angleError - angleErrorLast;

        cOutput = angleError * -100;
        angleErrorLast = angleError;

      }
      else{
        cOutput = 0;
      }

      joyYval = Robot.oi.main.getY(Hand.kLeft);
      joyXval = Robot.oi.main.getX(Hand.kRight);
      yOutput = 21000 * Exponential.exponential(joyYval, DriveExp, JoyDead, MotorMin);
      xOutput = 21000 * Exponential.exponential(joyXval, DriveExp, JoyDead, MotorMin);
      
      /*
      if(Math.abs(joyXval) > JoyDead){
        cOutput = 0;        
      } else{
        cOutput = chassis.getAngleRate() * kSkew;
      }
      */
      chassis.set(ControlMode.Velocity, (yOutput) - (xOutput) + cOutput, (yOutput) + xOutput - cOutput);
    }
    
  }

  
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
