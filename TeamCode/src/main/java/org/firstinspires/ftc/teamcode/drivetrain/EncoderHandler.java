package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderHandler {
    private final DcMotorEx parL, parR, perp;
    IMUHandler imuHandler;

    private static final double TICKS_PER_REV = 8192.0;
    private static final double WHEEL_RADIUS = 1.0; // inches
    private static final double GEAR_RATIO = 1.0;

    public double parLDist = 0;
    public double parRDist = 0;
    public double perpDist = 0;

    public int lastParL = 0;
    public int lastParR = 0;
    public int lastPerp = 0;

    public double deltaPar, deltaPerp;

    public EncoderHandler(HardwareMap hardwareMap) {
        parL = hardwareMap.get(DcMotorEx.class, "leftFront");
        parR = hardwareMap.get(DcMotorEx.class, "rightFront");
        perp = hardwareMap.get(DcMotorEx.class, "rightBack");

        initializeEncoders();
    }

    public void initializeEncoders() {
        parL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        parR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        parL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        parR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        perp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getParLTicks() {
        return parL.getCurrentPosition();
    }

    public int getParRTicks() {
        return parR.getCurrentPosition();
    }

    public int getPerpTicks() {
        return perp.getCurrentPosition();
    }

    public double ticksToInches() {
        return 2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO / TICKS_PER_REV;
    }

    public void getDeltas() {
        deltaPar = ((getParRTicks() * parLDist + getParLTicks() * parRDist)/(parRDist+parLDist)); // 2 PARALLEL PODS
        deltaPerp = (getPerpTicks() - lastPerp) * ticksToInches() - (imuHandler.deltaHeading() * perpDist);
    }


    public void updateLastTicks() {
        lastParL = getParLTicks();
        lastParR = getParRTicks();
        lastPerp = getPerpTicks();
    }
}
