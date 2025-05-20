package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.drivetrain.Drivetrain.State.IDLE;
import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PIDalgs;

import java.util.Vector;

public class Drivetrain {

    public enum State {
        GO_TO_POINT,
        DRIVE,
        BRAKE,
        HOLD_POINT,
        IDLE

    }
    public State state = IDLE;


    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    PIDalgs piDalgs;
    Localizer localizer;


    public Drivetrain(HardwareMap hardwareMap, Localizer localizer) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        this.localizer = localizer;
    }


    // TODO: add all functions in red and start making stuff happen!!


    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }
        // update things will do tomorrow

        // update localizer, Pose2d estimate, ROBOT_POSITION = new Pose2d(estimate.x, estimate.y, estimate.heading); ROBOT_VELOCITY might be applicable too
        // calculate errors, update telemetry.

        switch (state) {

            case GO_TO_POINT:
                setMinPowersToOvercomeFriction();
                PID();

                if (atPoint()) {
                    if (finalAdjustment) {
                        state = State.FINAL_ADJUSTMENT;
                    } else {
                        state = State.BRAKE;
                    }
                }
                break;

            case BRAKE:
                stopAllMotors();
                state = State.HOLD_POINT;
                break;

            case HOLD_POINT:
                if (!atPointThresholds(1.5, 1.5, 5)) {
                    state = State.GO_TO_POINT;
                }
                break;
            case DRIVE:
                break;
            case IDLE:
                break;


        }
    }

    public boolean atPointThresholds (double xThresh, double yThresh, double headingThresh) {
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(turnError) < Math.toRadians(headingThresh);
    }

    public void drive(Gamepad gamepad) {
        state = State.DRIVE;

        double forward = gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;

        setWeightedMotorPowers(forward, strafe, turn);
    }










    // Method to move the robot to a specific point (target position)
    // Takes in target position (X, Y), target orientation (T), speed, distance threshold, and the condition to move on
    /*
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


    } */
}
