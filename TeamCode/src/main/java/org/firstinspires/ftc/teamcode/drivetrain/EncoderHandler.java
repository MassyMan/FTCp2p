package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderHandler {
    private final DcMotorEx parL, parR, perp;
    IMUHandler imuHandler;
    EncoderSpecial encoderSpecial;

    public double lastParL, deltaParL, parLTicks;

    public void updateEncoders() {
        encoderSpecial.update(parL);
        lastParL = encoderSpecial.lastPosition;
        parLTicks = encoderSpecial.currentPosition;
        deltaParL = encoderSpecial.deltaInches;
    }


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

}
