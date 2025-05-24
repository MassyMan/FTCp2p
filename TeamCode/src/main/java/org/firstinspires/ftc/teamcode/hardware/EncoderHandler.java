package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderHandler {
    IMUHandler imuHandler;


    // gonna remove unused later
    public double lastParL, deltaParL, parLTicks;
    public double lastParR, deltaParR, parRTicks;
    public double lastPerp, deltaPerp, perpTicks;
    public double deltaParCombined;
    public double deltaHeading;

    public double parLDist = 0; // INCHES
    public double parRDist = 0;
    public double perpDist = 0;

    Encoder encoderParL, encoderParR, encoderPerp;


    public EncoderHandler (HardwareMap hardwareMap) {
        encoderParL = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        encoderParR = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        encoderPerp = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        imuHandler = new IMUHandler(hardwareMap);
    }

    public void update(double deltaHeading) {

        encoderParL.update();
        encoderParR.update();
        encoderPerp.update();

        deltaParCombined = ((encoderParR.getDelta() * parLDist + parLTicks * parRDist)/(parRDist+parLDist));
        deltaPerp = encoderPerp.getDelta() - deltaHeading * perpDist;




        /*
        lastParL = encoderParL.lastPosition;
        parLTicks = encoderParL.currentPosition;
        deltaParL = encoderParL.getDelta();


        lastParR = encoderParR.lastPosition;
        parRTicks = encoderParR.currentPosition;
        deltaParR = encoderParR.getDelta();

        encoderPerp.update();
        lastPerp = encoderPerp.lastPosition;
        perpTicks = encoderPerp.currentPosition;
        deltaPerp = encoderPerp.getDelta();
        deltaHeading = imuHandler.deltaHeading();
        deltaParCombined = ((parRTicks * parLDist + parLTicks * parRDist)/(parRDist+parLDist));
*/
    }

}
