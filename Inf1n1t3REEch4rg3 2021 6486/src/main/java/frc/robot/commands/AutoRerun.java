// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RerunSubsystem;
import frc.robot.subsystems.LimelightSubsystem.CameraMode;
import frc.robot.subsystems.LimelightSubsystem.LightMode;
import java.io.*;
// import java.util.Date;
public class AutoRerun extends CommandBase {
  private final DriveSubsystem chassis;
  private final LimelightSubsystem camera;
  private final RerunSubsystem rerun;

  // private static Date processDate = new Date();

  private double leftVal;
  private double rightVal;

  // Joy Code
  private double cOutput;


  /** Creates a new AutoRerun. */
  public AutoRerun(final DriveSubsystem sub1, final LimelightSubsystem sub2, final RerunSubsystem sub5) {
    // Use addRequirements() here to declare subsystem dependencies.
    chassis = sub1;
    camera = sub2;
    rerun = sub5;
    addRequirements(chassis);
    addRequirements(camera);
    addRequirements(rerun);
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


 
      camera.setLedMode(LightMode.eOff);

  
		   try {
			      String fileName="/home/lvuser/slalomPath8.txt";
	   	    	File file = new File(fileName);
	   	    	BufferedReader reader = null;
	   	    	reader = new BufferedReader(new FileReader(file));
	          String tempStr;
	          while ((tempStr = reader.readLine()) != null) {
	           	   String[] ret = tempStr.split(",");
                 leftVal=Double.parseDouble(ret[0]);
                 rightVal=Double.parseDouble(ret[1]);
                 SmartDashboard.putNumber("leftVal of auto", leftVal);
                 SmartDashboard.putNumber("rightVal of auto", rightVal);
                //  processDate = new Date();
                 chassis.set(ControlMode.Velocity, leftVal, rightVal);
                //  rerun.save(processDate+":"+leftVal+','+rightVal, "slalomPathAutoTime2.txt");
                 try
                 {
                      Thread.sleep(20);
                 }
                 catch(InterruptedException ex)
                 {
                      Thread.currentThread().interrupt();
                 }
	           }
	           reader.close();

		   } catch (IOException e) {  	   
	            System.out.print("Exception");
	       }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}