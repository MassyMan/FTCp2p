package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {

    EncoderHandler encoderHandler;
    IMUHandler imuHandler;

    public double currentHeading;
    public double lastHeading, lastPerp, lastParL, lastParR, deltaHeading;
    public double ticksToInches;
    public double x, y, heading;

    public int parLTicks, parRTicks, perpTicks;
    public double parLDist, parRDist, perpDist;

    public void setCurrentPosition(double X, double Y, double Heading){
        x = X;
        y = Y;
        heading = Heading;
    }

    public void updatePosition() {
        double dx = encoderHandler.deltaParCombined * imuHandler.cosHeading() - encoderHandler.deltaPerp * imuHandler.sinHeading();
        double dy = encoderHandler.deltaPerp * imuHandler.sinHeading() + encoderHandler.deltaPerp * imuHandler.cosHeading();

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
