package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IMUHandler {
    private final BNO055IMU imu;

    public IMUHandler(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;

    }
}
