package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {
    private final DcMotorEx parL, parR, perp;
    private final BNO055IMU imu;

    private double x = 0, y = 0, heading = 0;

    private int lastPar = 0, lastPerp = 0;
    private double lastHeading = 0;

    private double parLDist = 0; // PAR POD DISTANCE FROM CENTER AXIS IN INCHES
    private double parRDist = 0;
    private double perpDist = 0; // PERP POD DISTANCE

    private static final double TICKS_PER_REV = 8192.0;
    private static final double WHEEL_RADIUS = 1.0; // inches
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_TO_INCHES = 2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO / TICKS_PER_REV;

    public Localizer(HardwareMap hardwareMap) {
        parL = hardwareMap.get(DcMotorEx.class, "leftFront");
        parR = hardwareMap.get(DcMotorEx.class, "rightFront");
        perp = hardwareMap.get(DcMotorEx.class, "rightBack");


        parL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        parR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        parL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        parR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        lastHeading = getRawHeading();
    }
    //  OLD CODE: double deltaPar = (parTicks - lastPar) * TICKS_TO_INCHES - (deltaHeading * parDist);
    public void update() {
        int parLTicks = parL.getCurrentPosition();
        int parRTicks = parR.getCurrentPosition();
        int perpTicks = perp.getCurrentPosition();

        double currentHeading = getRawHeading();
        double deltaHeading = angleWrap(currentHeading - lastHeading);

        double deltaPar = ((parRTicks * parLDist + parLTicks * parRDist)/(parRDist+parLDist));
        double deltaPerp = (perpTicks - lastPerp) * TICKS_TO_INCHES - (deltaHeading * perpDist);

        lastPar = parTicks;
        lastPerp = perpTicks;
        lastHeading = currentHeading;

        double sinHeading = Math.sin(currentHeading);
        double cosHeading = Math.cos(currentHeading);

        double dx = deltaPar * cosHeading - deltaPerp * sinHeading;
        double dy = deltaPar * sinHeading + deltaPerp * cosHeading;

        x += dx;
        y += dy;
        heading = currentHeading;

        Pose2D robotPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    private double getRawHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    private double angleWrap(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }
}
