/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CSVFile extends CommandBase {

  private final DriveSubsystem chassis;
  public static File file;
  public static FileWriter writer;
  public static StringBuilder sb;

  public static int rows = 0;

  public CSVFile(final DriveSubsystem sub1) {
    chassis = sub1;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static void addRow(final String var1, final Boolean var2, final int var3) throws IOException {
    sb.append("" + var1);
    sb.append(',');
    sb.append("" + var2);
    sb.append(',');
    sb.append("" + var3);
    sb.append('\n');

    rows += 1;

    //writer.write(sb.toString());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //StringBuilder sb = new StringBuilder();
    String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
    File file = new File(
        "C:\\Users\\TenMoreTons Robotics\\Documents\\Filetests\\" + (timeStamp + "_" + "TEST") + ".csv");
    if(!file.exists()){
      file = new File(
        "C:\\Users\\TenMoreTons Robotics\\Documents\\Filetests\\" + (timeStamp + "_" + "FILEDNE") + ".csv");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("File Exists", file.exists());
      try {
        FileWriter writer = new FileWriter(file);
        StringBuilder sb = new StringBuilder();
        if(rows == 0){
          sb.append("var1");
          sb.append(',');
          sb.append("var2");
          sb.append(',');
          sb.append("var3");
          sb.append('\n');
          rows += 1;
          writer.write(sb.toString());
        }else if(rows < 30){
          sb.append("test");
          sb.append(',');
          sb.append(""+(rows % 2 == 0));
          sb.append(',');
          sb.append(""+rows);
          sb.append('\n');
          rows += 1;
          writer.write(sb.toString());
        }else if (rows > 30){
          writer.close();
        }
      } catch (IOException e) {
        DriverStation.reportError(""+e.getMessage(), true);
        e.printStackTrace();
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
