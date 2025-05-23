package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Globals {
    // general
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static RunMode RUNMODE = RunMode.TESTER;
    public static boolean isRed = false;
    // drivetrain
    public static double MIN_MOTOR_POWER_TO_OVERCOME_FRICTION = 0.1; // TODO: TUNE THIS WITH TUNING OPMODE
    public static boolean DRIVETRAIN_ENABLED = true;

    public static Pose2D ROBOT_POSITION = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
        return LOOP_TIME;
    }

}
