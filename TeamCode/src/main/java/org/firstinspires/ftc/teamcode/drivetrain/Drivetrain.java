package org.firstinspires.ftc.teamcode.drivetrain;


import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PIDalgs;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Vector;

public class Drivetrain {

    public Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    boolean brakeAtEnd;

    public enum State {
        GO_TO_POINT,
        DRIVE,
        BRAKE,
        HOLD_POINT,
        FINAL_ADJUSTMENT,
        IDLE

    }
    public State state = State.IDLE;


    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    PIDalgs piDalgs;
    Localizer localizer;


    public Drivetrain(HardwareMap hardwareMap, Localizer localizer) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        this.localizer = localizer;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // PORT 0 (parL)
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // PORT 2 (perp)
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // PORT 0 (parR)

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    // TODO: add all functions in red and start making stuff happen!!


    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }
        localizer.update();

        switch (state) {

            case GO_TO_POINT:
                PID();

                if (atPoint()) {
                    state = State.BRAKE;
                }


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
                state = State.IDLE;
                break;

            case DRIVE:
                break;
            case IDLE:
                break;


        }
    }

    double xError = ROBOT_POSITION.x - targetPose.getX;

    public boolean atPointThresholds (double xThresh, double yThresh, double headingThresh) {
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(turnError) < Math.toRadians(headingThresh);
    }

    public void goToPoint(Pose2D targetPoint, boolean brake) {
        targetPose = targetPoint;
        brakeAtEnd = brake;
        state = State.GO_TO_POINT;
    }

    public void driverControl(Gamepad gamepad) {
        state = State.DRIVE;
        double slowMode = 0.3;
        double forward = gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;
        if (gamepad.right_bumper) {
            forward = forward * slowMode;
            strafe = strafe * slowMode;
            turn = turn * slowMode;
        }
        setWeightedMotorPowers(forward, strafe, turn);
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) { // Raw power application
        leftFront.setPower(lf);
        leftBack.setPower(lr);
        rightBack.setPower(rr);
        rightFront.setPower(rf);
    }

    public void setWeightedMotorPowers(double forward, double strafe, double turn) { // Mecanum movement
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1); // Scaling
        double[] weightPowers = new double[]{
            (forward + strafe + turn) / denominator,
            (forward - strafe + turn) / denominator,
            (forward - strafe - turn) / denominator,
            (forward + strafe - turn) / denominator
        };
        setMotorPowers(weightPowers[0], weightPowers[1], weightPowers[2], weightPowers[3]);
    }

    public void stopAllMotors () {
        setMotorPowers(0,0,0,0);
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
