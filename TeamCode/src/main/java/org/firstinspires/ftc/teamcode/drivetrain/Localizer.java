package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {

    EncoderHandler encoderHandler;
    IMUHandler imuHandler;

    public double currentHeading;
    public double lastHeading, lastPerp, lastParL, lastParR;
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
        lastPerp = perpTicks;
        lastParL = parLTicks;
        lastParR = parRTicks;
        lastHeading = currentHeading;
        parLTicks = encoderHandler.getParLTicks();
        parRTicks = encoderHandler.getParRTicks();
        perpTicks = encoderHandler.getPerpTicks();
        parLDist = encoderHandler.parLDist;
        parRDist = encoderHandler.parRDist;
        perpDist = encoderHandler.perpDist;
        ticksToInches = encoderHandler.ticksToInches();

        currentHeading = imuHandler.getHeading();
        double deltaHeading = angleWrap(currentHeading - lastHeading);

        double deltaPar = ((parRTicks * parLDist + parLTicks * parRDist)/(parRDist+parLDist)); // 2 PARALLEL PODS
        double deltaPerp = (perpTicks - lastPerp) * ticksToInches - (deltaHeading * perpDist);

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
        return imuHandler.getHeading();
    }

    private double angleWrap(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }
}
