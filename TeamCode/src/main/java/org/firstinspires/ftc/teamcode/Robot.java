package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.HardwareQueue;
import org.firstinspires.ftc.teamcode.hardware.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class Robot {
    public HardwareQueue hardwareQueue;

    public final Sensors sensors;
    public final Drivetrain drivetrain;


    public Robot(HardwareMap hardwareMap) {
        hardwareQueue = new HardwareQueue();
        sensors = new Sensors(hardwareMap, hardwareQueue, this);
        drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors, this);

    }

    public void update() {
        START_LOOP();
        updateSubsystems();
    }

    public void updateSubsystems() {
        hardwareQueue.update();
        sensors.update();

        drivetrain.update();
        // deposit, intake, airplane, hang, droppers
    }

    public void goToPoint(Pose2D pose, boolean brakey, boolean finalAdjusty, double maximumPower) {

    }
}
