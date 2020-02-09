package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class InitializeTalon {

    public static void initRightDriveFalcon(final TalonFX motor){
        //Set Sensor Phase
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        motor.setSensorPhase(false);
        //Invert
        motor.setInverted(true);
        //Factory default hardware to prevent unexpected behavior
        motor.configFactoryDefault();
        //Output Settings
        motor.configNominalOutputForward(0, Constants.kTimeoutMs);
        motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
        motor.configPeakOutputForward(1, Constants.kTimeoutMs);
        motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        //Reset Encoder
        motor.setSelectedSensorPosition(0);
        //PIDF Config
        motor.config_kF(Constants.kPIDLoopIdx, Constants.kRightDrivekF, Constants.kTimeoutMs);
        motor.config_kP(Constants.kPIDLoopIdx, Constants.kRightDrivekP, Constants.kTimeoutMs);
        motor.config_kI(Constants.kPIDLoopIdx, Constants.kRightDrivekI, Constants.kTimeoutMs);
        motor.config_kD(Constants.kPIDLoopIdx, Constants.kRightDrivekD, Constants.kTimeoutMs);
    }

    public static void initLeftDriveFalcon(final TalonFX motor){
        //Set Sensor Phase
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        motor.setSensorPhase(false);
        //Factory default hardware to prevent unexpected behavior
        motor.configFactoryDefault();
        //Output Settings
        motor.configNominalOutputForward(0, Constants.kTimeoutMs);
        motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
        motor.configPeakOutputForward(1, Constants.kTimeoutMs);
        motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        //Reset Encoder
        motor.setSelectedSensorPosition(0);
        //PIDF Config
        motor.config_kF(Constants.kPIDLoopIdx, Constants.kLeftDrivekF, Constants.kTimeoutMs);
        motor.config_kP(Constants.kPIDLoopIdx, Constants.kLeftDrivekP, Constants.kTimeoutMs);
        motor.config_kI(Constants.kPIDLoopIdx, Constants.kLeftDrivekI, Constants.kTimeoutMs);
        motor.config_kD(Constants.kPIDLoopIdx, Constants.kLeftDrivekD, Constants.kTimeoutMs);
    }

    public static void initFWMotor(final TalonFX motor){
        //Factory default hardware to prevent unexpected behavior
        motor.configFactoryDefault();
        //Set Sensor Phase
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        motor.setSensorPhase(false);
        //Brake Mode
        motor.setNeutralMode(NeutralMode.Coast);
        //Output Settings
        motor.configNominalOutputForward(0, Constants.kTimeoutMs);
        motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
        motor.configPeakOutputForward(1, Constants.kTimeoutMs);
        motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        //PIDF Config
        motor.config_kF(Constants.kPIDLoopIdx, Constants.kFlywheelkF, Constants.kTimeoutMs);
        motor.config_kP(Constants.kPIDLoopIdx, Constants.kFlywheelkP, Constants.kTimeoutMs);
        motor.config_kI(Constants.kPIDLoopIdx, Constants.kFlywheelkI, Constants.kTimeoutMs);
        motor.config_kD(Constants.kPIDLoopIdx, Constants.kFlywheelkD, Constants.kTimeoutMs);
    }

}