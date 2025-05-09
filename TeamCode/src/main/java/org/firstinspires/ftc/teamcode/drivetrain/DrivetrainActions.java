package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;


public class DrivetrainActions (double targetX, double targetY, double targetT, double moveSpeed, boolean remainAfter) {


    PIDalgs piDalgs;

    Localizer localizer;

    public void gotoPoint(double targetX, double targetY, double targetT, double moveSpeed, boolean remainAfter) {
        DcMotorEx leftFront, leftBack, rightBack, rightFront;
        piDalgs.runPID(targetX, targetY, targetT);

            


    }

}
