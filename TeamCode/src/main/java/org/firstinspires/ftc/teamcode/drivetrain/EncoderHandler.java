package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderHandler {
    IMUHandler imuHandler;



    public double lastParL, deltaParL, parLTicks;
    public double lastParR, deltaParR, parRTicks;
    public double lastPerp, deltaPerp, perpTicks;
    public double deltaParCombined;
    public double deltaHeading;
    public double parLDist = 0; // INCHES
    public double parRDist = 0;

    Encoder encoderParL, encoderParR, encoderPerp;
 //   EncoderSpecial encoderParR = new EncoderSpecial (hardwareMap.get(DcMotorEx.class, "rightFront")); // PORT 3

    public EncoderHandler (HardwareMap hardwareMap) {
        encoderParL = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        encoderParR = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        encoderPerp = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));

        imuHandler = new IMUHandler(hardwareMap);
    }

    public void updateData() {

        encoderParL.update();
     //   lastParL = encoderParL.lastPosition;
        parLTicks = encoderParL.currentPosition;
        deltaParL = encoderParL.getDelta();

        encoderParR.update();
    //    lastParR = encoderParR.lastPosition;
        parRTicks = encoderParR.currentPosition;
        deltaParR = encoderParR.getDelta();

        encoderPerp.update();
    //    lastPerp = encoderPerp.lastPosition;
        perpTicks = encoderPerp.currentPosition;
        deltaPerp = encoderPerp.getDelta();
        deltaHeading = imuHandler.deltaHeading();
        deltaParCombined = ((parRTicks * parLDist + parLTicks * parRDist)/(parRDist+parLDist)); // PULLED TO LOCALIZER

    }

}
