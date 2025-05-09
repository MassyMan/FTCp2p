package org.firstinspires.ftc.teamcode.drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class EncoderSpecial {
    EncoderHandler encoderHandler;
    public static final double TICKS_PER_INCH = 0; // I am going to measure this manually RR style with tape measure test
    // OLD TICKS PER INCH: 2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO / TICKS_PER_REV;

    DcMotorEx Encoder;
    public EncoderSpecial(DcMotorEx dcMotorEx) {
        Encoder = dcMotorEx;
        Encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int currentPosition = 0;
    public int lastPosition = 0;

    public void update() {
        lastPosition = currentPosition;
        currentPosition = Encoder.getCurrentPosition();
    }

    public double getDelta() {
        return (currentPosition - lastPosition) * TICKS_PER_INCH;
    }


}
