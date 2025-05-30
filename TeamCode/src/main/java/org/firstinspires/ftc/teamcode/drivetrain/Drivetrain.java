package org.firstinspires.ftc.teamcode.drivetrain;


import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.HardwareQueue;
import org.firstinspires.ftc.teamcode.hardware.PriorityMotor;
import org.firstinspires.ftc.teamcode.hardware.Sensors;
import org.firstinspires.ftc.teamcode.utils.Localizer;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.PIDalgs;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.Arrays;
import java.util.List;

public class Drivetrain {



    public enum State {
        GO_TO_POINT,
        DRIVE,
        BRAKE,
        HOLD_POINT, // remove possibly?
        FINAL_ADJUSTMENT,
        IDLE

    }
    public State state = State.IDLE;


//    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    PIDalgs piDalgs;
    Localizer localizer;

    public PriorityMotor leftFront, leftBack, rightBack, rightFront;
    private List<PriorityMotor> motors;

    private HardwareQueue hardwareQueue;
    private Sensors sensors;
    public Robot robot;
/*
 // TODO: SINGLETON CLASS WITH ALL OF THIS GARBAGE INSIDE OF IT
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


 */
public Drivetrain(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
    this.hardwareQueue = hardwareQueue;
    this.sensors = sensors;
    this.robot = robot;

    leftFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftFront"),
            "leftFront",
            3, 5, sensors
    );

    leftBack = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftRear"),
            "leftRear",
            3, 5, sensors
    );
    rightBack = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightRear"),
            "rightRear",
            3, 5, sensors
    );
    rightFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightFront"),
            "rightFront",
            3, 5, sensors
    );

    motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

    for (PriorityMotor motor : motors) {
        MotorConfigurationType motorConfigurationType = motor.motor[0].getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.motor[0].setMotorType(motorConfigurationType);

        hardwareQueue.addDevice(motor);
    }

    // resetting odos
    leftFront.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBack.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBack.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    leftFront.motor[0].setDirection(DcMotor.Direction.REVERSE);
    leftBack.motor[0].setDirection(DcMotor.Direction.REVERSE);

}

    public double xError, yError, turnError;
    public double xThreshold, yThreshold, hThreshold;
    PID xPD = new PID(0, 0);
    PID yPD = new PID(0,0);
    PID hPD = new PID(0,0);


    public void update() {
        localizer.update();

        switch (state) {

            case GO_TO_POINT:
                getErrors();
                PIDmonster();

                if (atPoint()) {
                    if (finalAdjust) {
                        state = State.FINAL_ADJUSTMENT;
                    } else {
                        state = State.BRAKE;
                    }
                }
                break;

            case FINAL_ADJUSTMENT: // TODO: make this a thing possibly
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

    public boolean atPoint() { // Thresholds for determining atPoint
        return Math.abs(xError) < xThreshold && Math.abs(yError) < yThreshold && Math.abs(turnError) < Math.toRadians(hThreshold);
    }

    public void PIDmonster() {
        double fwd, strafe, turn, turnAdjustThreshold;
        fwd = Math.abs(xError) > xThreshold/2 ? xPD.update(xError, -maxPower, maxPower) + 0.05 * Math.signum(xError) : 0;
        strafe = Math.abs(yError) > yThreshold/2 ? yPD.update(yError, -maxPower, maxPower) + 0.05 * Math.signum(yError) : 0;

        turnAdjustThreshold = (Math.abs(xError) > xThreshold/2 || Math.abs(yError) > yThreshold/2) ? hThreshold/3.0 : hThreshold;
        turn = Math.abs(turnError) > Math.toRadians(turnAdjustThreshold)/2 ? hPD.update(turnError, -maxPower, maxPower) : 0;

        setWeightedMotorPowers(strafe, fwd, turn); // APPLY MOTOR POWERS YAY
    }


    public Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private Pose2D lastTarget = targetPose;
    boolean brake, finalAdjust;
    public double targetX, targetY, targetT;
    double maxPower = 1.0;

    public void goToPoint(Pose2D targetPoint, boolean brake, boolean finalAdjust, double maxPower) {
      //  if (targetPoint.getX(DistanceUnit.INCH) != lastTarget.getX(DistanceUnit.INCH) ||
         //   targetPoint.getY(DistanceUnit.INCH) != lastTarget.getY(DistanceUnit.INCH) ||
        //    targetPoint.getHeading(AngleUnit.DEGREES) != lastTarget.getHeading(AngleUnit.DEGREES)) { // IF THE POINT IS DIFFERENT, GO TO POINT; ELSE DON'T
        if (targetPoint != lastTarget) {
            targetPose = targetPoint;
            lastTarget = targetPose; // RESET LAST TARGET TO CURRENT TARGET
            targetX = targetPose.getX(DistanceUnit.INCH);
            targetY = targetPose.getY(DistanceUnit.INCH);
            targetT = targetPose.getHeading(AngleUnit.DEGREES);

            this.brake = brake;
            this.finalAdjust = finalAdjust;
            this.maxPower = Math.abs(maxPower);

            state = State.GO_TO_POINT;
        }


    }

    public void getErrors() {
        turnError = Utils.headingClip(targetT - localizer.heading);
        xError = (targetX - localizer.x) * Math.cos(turnError) - (targetY - localizer.y) * Math.sin(turnError);
        yError = (targetY - localizer.y) * Math.cos(turnError) + (targetX - localizer.x) * Math.sin(turnError);
    }

    public void driverControl(Gamepad gamepad) {
        state = State.DRIVE;
        double slowMode = 0.3;
        double forward = gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;

        if (gamepad.right_bumper) { // Right bumper turns on/off slow mode
            forward *= slowMode;
            strafe *= slowMode;
            turn *= slowMode;
        }
        setWeightedMotorPowers(forward, strafe, turn);
    }


    // MOTOR CONFIGURATION SETTINGS

    public void setMotorPowers(double lf, double lr, double rr, double rf) { // Raw power application
        leftFront.setPower(lf);
        leftBack.setPower(lr);
        rightBack.setPower(rr);
        rightFront.setPower(rf);
    }

    public void setWeightedMotorPowers(double x, double y, double t) { // Mecanaum movement
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(t), 1); // Scaling
        double[] weightPowers = new double[]{
            (y + x + t) / denominator,
            (y - x + t) / denominator,
            (y - x - t) / denominator,
            (y + x - t) / denominator
        };
        setMotorPowers(weightPowers[0], weightPowers[1], weightPowers[2], weightPowers[3]);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setMode(runMode);
        }
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
