package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUHandler {
    private final BNO055IMU imu;
    private double lastHeading = 0.0;
    private double currentHeading = 0.0;

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

    public double sinHeading() {
        return Math.sin(currentHeading);
    }

    public double cosHeading() {
        return Math.cos(currentHeading);
    }

    public double getLastHeading() {
        return lastHeading;
    }

    public double deltaHeading() {
        return angleWrap(currentHeading - lastHeading);
    }

    private double angleWrap(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }
}
