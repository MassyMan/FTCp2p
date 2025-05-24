package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.AngleWrap;

public class IMUHandler {
    private final BNO055IMU imu;
    private double lastHeading = 0.0;
    private double currentHeading = 0.0;

    AngleWrap mathUtils;

    public IMUHandler(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        update(); // Set initial heading
    }

    public void update() {
        lastHeading = currentHeading;
        currentHeading = imu.getAngularOrientation().firstAngle;
    }


    public double getHeading() {
        return currentHeading;
    }
    public double getLastHeading() {
        return lastHeading;
    }
    public double deltaHeading() {
        return AngleWrap.angleWrap(currentHeading - lastHeading);
    }

}
