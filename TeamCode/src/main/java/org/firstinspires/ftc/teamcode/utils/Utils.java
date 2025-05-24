package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// COPY PASTED FROM CLUELESS CENTERSTAGE :skull:
// https://github.com/FTCclueless/Centerstage/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/Utils.java


// NOT IN USE AT THE MOMENT [EXCEPT IN PID.java (also not in use)]

public class Utils {
    public Utils() {}

    public static double minMaxClip(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }

    public static int minMaxClipInt(double value, double min, double max) {
        return (int) Math.min(Math.max(min, value), max);
    }

    public static double headingClip(double value) {
        while(value >= Math.PI) {
            value -= 2*Math.PI;
        }
        while(value <= -Math.PI) {
            value += 2*Math.PI;
        }
        return value;
    }

    public static boolean withinThreshold(double value, double minThreshold, double maxThreshold) {
        return value > minThreshold && value < maxThreshold;
    }

    public static double kalmanFilter (double value1, double value2, double value2Weight) {
        return (value1 * (1.0-value2Weight)) + (value2 * value2Weight);
    }

    public static double calculateDistanceBetweenPoints (Pose2D point1, Pose2D point2) {
        double deltaX = point1.getX(DistanceUnit.INCH) - point2.getX(DistanceUnit.INCH);
        double deltaY = point1.getY(DistanceUnit.INCH) - point2.getY(DistanceUnit.INCH);
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
}