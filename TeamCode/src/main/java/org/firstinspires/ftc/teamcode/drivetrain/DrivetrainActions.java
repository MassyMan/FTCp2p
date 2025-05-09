package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DrivetrainActions {
    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    PIDalgs piDalgs;
    Localizer localizer;

    public boolean moveOn;

    // Constructor to initialize motors and localizer
    public DrivetrainActions(HardwareMap hardwareMap, Localizer localizer) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        this.localizer = localizer;
    }

    // Method to move the robot to a specific point (target position)
    // Takes in target position (X, Y), target orientation (T), speed, distance threshold, and the condition to move on
    public void gotoPoint(double targetX, double targetY, double targetT, double moveSpeed, double distThreshold, boolean condition) {
        moveOn = false;

        // Run the PID control to get the motor powers for moving toward the target
        piDalgs.runPID(targetX, targetY, targetT);

        // Set motor powers based on the PID controller output
        leftFront.setPower(piDalgs.lfPower);
        leftBack.setPower(piDalgs.lbPower);
        rightBack.setPower(piDalgs.rbPower);
        rightFront.setPower(piDalgs.rfPower);

        // Calculate the error (distance between current position and target)
        double error = Math.sqrt(Math.pow(Math.abs(localizer.x - targetX), 2) + Math.pow(Math.abs(localizer.y - targetY), 2));

        // If the robot is within the threshold distance and the condition is met, allow moving to the next action
        if (error < distThreshold && condition) {
            moveOn = true; // Allow next action to proceed
        }
    }
}
