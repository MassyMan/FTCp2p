package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

// COPIED FROM CLUELESS CENTERSTAGE

// https://www.ctrlaltftc.com/the-pid-controller

public class PID {
    public double p;
    public double i;
    public double d;
    public PID(double P, double D){
        p=P;
   //     i=I;
        d=D;
    }
    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    int counter = 0;
    double loopTime = 0.0;

    public void resetIntegral() {
        integral = 0;
    }

    public double update(double error, double min, double max){
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }

        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime; // lastLoopTime's start time

        double proportion = p * error;
      //  integral += error * i * loopTime;
        double derivative = d * (error - lastError)/loopTime;

        lastError = error;
        counter ++;

        return Utils.minMaxClip(proportion + derivative, min, max);
    }

    public void updatePID(double p, double d) {
        this.p = p;
   //     this.i = i;
        this.d = d;
    }
}