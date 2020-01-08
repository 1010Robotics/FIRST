package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import edu.wpi.first.wpilibj.DriverStation;

public class CSVFile {

    static StringBuilder sb = new StringBuilder();
    static PrintWriter writer;
    public int size;
    public int rows;
    private static int rowCount = 1;

    public String createFileName(String name) {

        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
        return timeStamp + "_" + name;

    }

    public static void addRow(int output, int preOutput, String batteryID, int batteryLevel) {

        //Adds Lines to our CSV. This will be called in the execute() portion of a Command
        sb.append("" + output);
        sb.append(',');
        sb.append(preOutput);
        sb.append(',');
        sb.append(batteryID);
        sb.append(',');
        sb.append(batteryLevel);
        sb.append('\n');
        rowCount += 1;

    }

    public static void createCSV(String filename) {

    //creates a CSV for a Given Filename. This will be called in the Subsystem.
        try (PrintWriter writer = new PrintWriter(new File(filename + ".csv"))) {

            sb.append("output");
            sb.append(',');
            sb.append("preOutput");
            sb.append(',');
            sb.append("batteryID");
            sb.append(',');
            sb.append("batteryLevel");
            sb.append('\n');
        }

        catch (FileNotFoundException ex) {
            DriverStation.reportError("PrintWriter not Initialized: "+ex, true);
        }
    }

    public static void closeCSV(String filename) {

        writer.write(sb.toString());

    }


    public CSVFile(String filename) {

        createCSV(createFileName(filename));
        this.size = sb.length();
        this.rows = rowCount;

    }
}

