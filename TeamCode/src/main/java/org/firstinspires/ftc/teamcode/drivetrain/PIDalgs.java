package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

// TODO: APPLY Motor powers in the autonomous mode class, just pull the numbers from here


public class PIDalgs {

    Localizer localizer;

    public double kP, kI, kD; // AXIAL (X/Y) CONSTANTS
    public double hP, hI, hD; // THETA (HEADING) CONSTANTS

    public double pX, pY, pT; // previous errors

    public double targetX, targetY, targetT;
    public double moveSpeed;

    public double axialThreshold; // Threshold for moving on to next path (X/Y Axis)
    public double thetaThreshold; // Threshold for heading
    public boolean atTarget;

    public void runPID () {
        double currentX = localizer.getX();
        double currentY = localizer.getY();
        double currentT = localizer.getX();

        ElapsedTime timer = new ElapsedTime();

        double deltaTime = timer.seconds(); // Delta for time
        timer.reset();

        double eX = targetX - currentX;
        double eY = targetY - currentY;
        double eT = targetT - currentT;

        double dX = (eX - pX) / deltaTime;
        double dY = (eY - pY) / deltaTime;
        double dT = (eT - pT) / deltaTime;

        double powerX = kP * eX + kI + kD * dX;
        double powerY = kP * eY + kI + kD * dY;
        double powerT = hP * eT + hI + hD * dT;

        double denominator = Math.max(Math.abs(powerY) + Math.abs(powerX) + Math.abs(powerT), 1); // Scaling

        double lfPower = Math.min((powerY + powerX + powerT) / denominator, 1);
        double lbPower = Math.min((powerY - powerX + powerT) / denominator, 1);
        double rfPower = Math.min((powerY - powerX - powerT) / denominator, 1);
        double rbPower = Math.min((powerY + powerX - powerT) / denominator, 1);

    }



}
