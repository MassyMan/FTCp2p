package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderHandler {
    IMUHandler imuHandler;



    public double lastParL, deltaParL, parLTicks;
    public double lastParR, deltaParR, parRTicks;
    public double lastPerp, deltaPerp, perpTicks;
    public double deltaHeading;
    public double deltaParCombined;

    public double parLDist = 0; // INCHES
    public double parRDist = 0;
    public double perpDist = 0;

    EncoderSpecial encoderParL, encoderParR, encoderPerp;
 //   EncoderSpecial encoderParR = new EncoderSpecial (hardwareMap.get(DcMotorEx.class, "rightFront")); // PORT 3

    public EncoderHandler (HardwareMap hardwareMap) {
        encoderParL = new EncoderSpecial (hardwareMap.get(DcMotorEx.class, "leftFront"));
        encoderParR = new EncoderSpecial (hardwareMap.get(DcMotorEx.class, "rightFront"));
        encoderPerp = new EncoderSpecial (hardwareMap.get(DcMotorEx.class, "rightBack"));
    }

    public void updateData() {


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
