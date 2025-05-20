package org.firstinspires.ftc.teamcode.utils;

public class Globals {
    // general
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static RunMode RUNMODE = RunMode.TESTER;
    public static boolean isRed = false;
    // drivetrain
    public static double MIN_MOTOR_POWER_TO_OVERCOME_FRICTION = 0.1; // TODO: TUNE THIS WITH TUNING OPMODE
    public static boolean DRIVETRAIN_ENABLED = true;

    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
        return LOOP_TIME;
    }

}
