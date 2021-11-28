package org.firstinspires.ftc.teamcode.differential.philobots;

import org.firstinspires.ftc.teamcode.differential.philobots.math.Vector2d;

public class RobotUtil {


    //deadband for joysticks
    public static double DEADBAND_MAG = 0.1;
    public static Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);


    //returns zero vector if joystick is within deadband
    public static Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return Vector2d.ZERO;
    }


    public static double scaleVal (double input, double minInputVal, double maxInputVal, double minOutputVal, double maxOutputVal) {
        if (input > maxInputVal) input = maxInputVal;
        double inputRange = Math.abs(maxInputVal - minInputVal);
        double outputRange = Math.abs(maxOutputVal - minOutputVal);
        double scaleFactor = input/inputRange;
        return outputRange * scaleFactor + minOutputVal;
    }
}
