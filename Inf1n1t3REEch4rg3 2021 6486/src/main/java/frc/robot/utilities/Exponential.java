package frc.robot.utilities;

public class Exponential{

    /**
     * @author Pedro Amui, Caden Hewlett
     * 
     * Create Exponential Curve for Driver Input when Driving
     * 
     * @param joystickVal Input Joystick Value
     * @param exPower Power for the exponential function (Commonly between 1 and 2) 
     * @param joyDead Deadzone for Joystick (Commonly 5% of Joystick Input)
     * @param motorMin Minimum Motor Output to Start Moving
     * @return Returns Motor Output with an Exponential Curve
     */
    public static double exponential(final double joystickVal, final double exPower, final double joyDead,final double motorMin) {
        double joySign;
        final double joyMax = 1 - joyDead;
        final double joyLive = Math.abs(joystickVal) - joyDead;
        if (joystickVal > 0) { joySign = 1; }
        else if (joystickVal < 0) { joySign = -1; }
        else { joySign = 0; }
        double power = (joySign * (motorMin + ((1 - motorMin) * (Math.pow(joyLive, exPower) / Math.pow(joyMax, exPower)))));
        if (Double.isNaN(power)) { power = 0; }
        return power;
    }

}