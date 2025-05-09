package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderHandler {
    IMUHandler imuHandler;
    HardwareMap hardwareMap;

    public double lastParL, deltaParL, parLTicks;
    public double lastParR, deltaParR, parRTicks;
    public double lastPerp, deltaPerp, perpTicks;
    public double deltaHeading;
    public double deltaParCombined;

    public double parLDist = 0; // INCHES
    public double parRDist = 0;
    public double perpDist = 0;

    public void updateData() {
        EncoderSpecial encoderParL = new EncoderSpecial (hardwareMap.get(DcMotorEx.class, "leftFront")); // PORT 0
        EncoderSpecial encoderParR = new EncoderSpecial (hardwareMap.get(DcMotorEx.class, "rightFront")); // PORT 3
        EncoderSpecial encoderPerp = new EncoderSpecial (hardwareMap.get(DcMotorEx.class, "rightBack")); // PORT 2

        encoderParL.update();
        lastParL = encoderParL.lastPosition;
        parLTicks = encoderParL.currentPosition;
        deltaParL = encoderParL.getDelta();

        encoderParR.update();
        lastParR = encoderParR.lastPosition;
        parRTicks = encoderParR.currentPosition;
        deltaParR = encoderParR.getDelta();

        encoderPerp.update();
        lastPerp = encoderPerp.lastPosition;
        perpTicks = encoderPerp.currentPosition;

        deltaParCombined = ((parRTicks * parLDist + parLTicks * parRDist)/(parRDist+parLDist)); // PULLED TO LOCALIZER
        deltaPerp = (perpTicks - lastPerp) * EncoderSpecial.TICKS_PER_INCH - (imuHandler.deltaHeading() * perpDist); // PULLED TO LOCALIZER

    }

}
