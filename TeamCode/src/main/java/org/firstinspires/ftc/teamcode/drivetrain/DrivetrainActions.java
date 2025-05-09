package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class DrivetrainActions {
    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public DrivetrainActions (HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
    }

    PIDalgs piDalgs;

    Localizer localizer;

    public double distThreshold = 1; // INCHES

    public void gotoPoint(double targetX, double targetY, double targetT, double moveSpeed, boolean remainAfter) {

        piDalgs.runPID(targetX, targetY, targetT);
        leftFront.setPower(piDalgs.lfPower);
        leftBack.setPower(piDalgs.lbPower);
        rightBack.setPower(piDalgs.rbPower);
        rightFront.setPower(piDalgs.rfPower);

        Pose2D currentPose = new Pose2D(DistanceUnit.INCH, localizer.x, localizer.y, AngleUnit.RADIANS, localizer.heading);
        Pose2D targetPose = new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.RADIANS, targetT);

        double error = Math.sqrt(Math.abs(Math.pow(Math.abs(localizer.x - targetX), 2) + Math.pow(Math.abs(localizer.y - targetY), 2)));

        if (error < distThreshold)) {
            // break out of here somehow and go to next action
        }

    }

}
