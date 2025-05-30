package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

// TODO: APPLY Motor powers in the autonomous mode class

// THIS CODE IS DOODOO


public class PIDalgs {

    Localizer localizer;
    ElapsedTime timer = new ElapsedTime();
    private static final double kP = 0; // AXIAL TUNING CONSTANTS
    private static final double kD = 0;

    private static final double hP = 0; // HEADING TUNING CONSTANTS
    private static final double hD = 0;

    private double eX, eY, eT; // current errors

    public double lfPower, lbPower, rfPower, rbPower;

    public void runPID (double targetX, double targetY, double targetT) {
        double currentX = localizer.getX();
        double currentY = localizer.getY();
        double currentT = localizer.getX();

        double deltaTime = timer.seconds(); // Delta for time
        timer.reset();

        double pX = eX; // previous errors
        double pY = eY;
        double pT = eT;

        eX = targetX - currentX; // current errors
        eY = targetY - currentY;
        eT = targetT - currentT;



        double dX = (eX - pX) / deltaTime; // deltas for errors
        double dY = (eY - pY) / deltaTime;
        double dT = (eT - pT) / deltaTime;

        double powerX = kP * eX + kD * dX;
        double powerY = kP * eY + kD * dY; // Forward
        double powerT = hP * eT + hD * dT;



    //    double denominator = Math.max(Math.abs(powerY) + Math.abs(powerX) + Math.abs(powerT), 1); // Scaling
/*
        lfPower = (powerY + powerX + powerT) / denominator; // motor powers
        lbPower = (powerY - powerX + powerT) / denominator;
        rfPower = (powerY - powerX - powerT) / denominator;
        rbPower = (powerY + powerX - powerT) / denominator;

 */
    }



}
