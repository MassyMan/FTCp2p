package org.firstinspires.ftc.teamcode.drivetrain;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class EncoderSpecial {
    EncoderHandler encoderHandler;
    private static final double TICKS_PER_REV = 8192.0;
    private static final double WHEEL_RADIUS = 1.0; // inches
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_INCH = 0; // I am going to measure this manually RR style with tape measure test
    // OLD TICKS PER INCH: 2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO / TICKS_PER_REV;

    DcMotorEx Encoder;
    public EncoderSpecial(DcMotorEx dcMotorEx) {
        Encoder = dcMotorEx;
    }

    public int currentPosition = 0;
    public int lastPosition = 0;
    public double deltaInches = 0;

    public void update(DcMotorEx dcMotorEx) {
        lastPosition = currentPosition;
        currentPosition = Encoder.getCurrentPosition();
        deltaInches = (currentPosition - lastPosition) * TICKS_PER_INCH;
    }
/*
    public double getDelta (DcMotorEx dcMotorEx) {
        return (currentPosition - lastPosition) * TICKS_PER_INCH;
    }

 */
}
