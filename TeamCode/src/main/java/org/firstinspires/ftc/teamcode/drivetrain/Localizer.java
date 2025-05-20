package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {

    EncoderHandler encoderHandler;
    IMUHandler imuHandler;

    public double currentHeading;
    public double x, y, heading;

    public Localizer (HardwareMap hardwareMap) {
        encoderHandler = new EncoderHandler(hardwareMap);
        imuHandler = new IMUHandler(hardwareMap);
    }

    public void setCurrentPosition(double X, double Y, double Heading){
        x = X;
        y = Y;
        heading = Heading;
    }

    public void updatePosition() {
        encoderHandler.update(imuHandler.deltaHeading());
        imuHandler.update();

        double dx = encoderHandler.deltaParCombined * Math.cos(imuHandler.getLastHeading()) - encoderHandler.deltaPerp * Math.sin(imuHandler.getLastHeading());
        double dy = encoderHandler.deltaPerp * Math.cos(imuHandler.getLastHeading()) + encoderHandler.deltaPerp * Math.cos(imuHandler.getLastHeading());

        x += dx;
        y += dy;
        heading = currentHeading;

        Pose2D robotPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    private double getRawHeading() {
        return imuHandler.getHeading();
    }


}
