package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// TODO: APPLY MOotr powers in the autonomous mode class, just pull the numbers from here
public class P2Point {



    public double kP, kI, kD; // AXIAL (X/Y) CONSTANTS
    public double hP, hI, hD; // THETA (HEADING) CONSTANTS

    public double pX, pY, pT; // previous errors

    public double targetX, targetY, targetT;
    public double moveSpeed;

    public double axialThreshold; // Threshold for moving on to next path (X/Y Axis)
    public double thetaThreshold; // Threshold for heading
    public boolean atTarget;

    public void runPID () {
        double currentX = ;
        double currentY = ;
        double currentT = ;

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

        double denominator = Math.max(Math.abs(powerY) + Math.abs(powerX) + Math.abs(powerT), 1);

        double lfPower = (powerY + powerX + powerT) / denominator;
        double lbPower = (powerY - powerX + powerT) / denominator;
        double rfPower = (powerY - powerX - powerT) / denominator;
        double rbPower = (powerY + powerX - powerT) / denominator;



    }



}
