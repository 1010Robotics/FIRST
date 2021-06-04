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

  private double angle1;
  private double error;
  private double angle2=0;
  private double tx;
  private boolean tv;
  private double leftCommand;
  private double rightCommand;
  double steeringAdjust = 0.0;

  // Exponential Variables
  private final double JoyDead = 0.050;
  private final double DriveExp = 1.5;
  private final double MotorMin = 0.008;
  private double getDistance;
  private double h1=1.75;
  private double h2=7.5;
  private double a1=40;
  private double a2;

  // Joy Code
  private double joyYval;
  private double joyXval;
  private double yOutput;
  private double xOutput;

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
    camera.setLedMode(LightMode.eOn);
    camera.setCameraMode(CameraMode.eVision);
    chassis.resetAngle();
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

    /**
     * AIMER
     */

    if (Robot.oi.main.getBumper(Hand.kLeft)) {

      tx=camera.getTx();
      tv=camera.isTarget();
      a2 = camera.getTy();
      getDistance = (h2-h1) / Math.tan((a1+a2)*Math.PI/180);
      if (tv == false)
      {
          // We don't see the target, seek for the target by spinning in the same place at a safe speed.
          steeringAdjust = 1800;
          leftCommand=steeringAdjust;
          rightCommand=-steeringAdjust;
          chassis.set(ControlMode.Velocity, rightCommand, leftCommand);
      }
      else
      { 
        if(tx>=-2&&tx<=2){
          //the target is within acceptable range
          if(getDistance>11){
            steeringAdjust = 1000+Math.pow((getDistance-11),1.4)*1000;
            chassis.set(ControlMode.Velocity, steeringAdjust, steeringAdjust-200);
          }else if(getDistance<10.8){
            steeringAdjust = 1000+Math.pow((11-getDistance),1.4)*1000;
            chassis.set(ControlMode.Velocity, -steeringAdjust, -steeringAdjust+200);
          }else{
            chassis.set(ControlMode.Velocity, 0, 0);
          }
          }else{
          // We do see the target, execute aiming code
          if(tx>0){
          steeringAdjust =  tx * 30 + 900;
          leftCommand=steeringAdjust;
          rightCommand=-steeringAdjust;
          chassis.set(ControlMode.Velocity, rightCommand, leftCommand);
          }else{
          steeringAdjust = tx * 30 - 900;
          leftCommand=steeringAdjust;
          rightCommand=-steeringAdjust;
          chassis.set(ControlMode.Velocity, rightCommand, leftCommand);
          }
          }
      
    }
  
    }

    /**
     * DRIVE
     */

    else {
      joyYval = Robot.oi.main.getY(Hand.kLeft);
      joyXval = Robot.oi.main.getX(Hand.kRight);
      yOutput = 21000 * Exponential.exponential(joyYval, DriveExp, JoyDead, MotorMin);
      xOutput = 21000 * Exponential.exponential(joyXval, DriveExp, JoyDead, MotorMin);
      if((-(yOutput) - (xOutput))==(-(yOutput) + (xOutput))){
        angle1 = chassis.getAngle();
        if(a2==0){
        angle2=angle1;
        }else{
          error=angle2-angle1;
          angle2=angle1;
        }
        
        //problem: how does the robot know which way is it going?
        //constantly comparing feedback
        if(error>1){
        chassis.set(ControlMode.Velocity,  ((yOutput) + (xOutput)), 0.01*error*((yOutput) - (xOutput)));
        }else if(error<-1){
        chassis.set(ControlMode.Velocity,  0.01*error*((yOutput) + (xOutput)), ((yOutput) - (xOutput)));
        }else{
        chassis.set(ControlMode.Velocity,  (yOutput) + (xOutput), +(yOutput) - (xOutput));
        }
      }else{
      chassis.set(ControlMode.Velocity,  -(yOutput) - (xOutput), -(yOutput) + (xOutput));
      }
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
