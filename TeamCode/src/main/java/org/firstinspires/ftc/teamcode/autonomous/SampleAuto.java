package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainActions;
import org.firstinspires.ftc.teamcode.drivetrain.EncoderHandler;
import org.firstinspires.ftc.teamcode.drivetrain.Localizer;

@Disabled
@Autonomous (name = "Sample Auto [PID-2-POINT]")
public class SampleAuto extends LinearOpMode {

    Localizer localizer;
    DrivetrainActions dtAction;


    @Override
    public void runOpMode() {

        EncoderHandler encoderHandler = new EncoderHandler(hardwareMap);


        localizer.setCurrentPosition(0, 0, Math.toRadians(90));

        dtAction.gotoPoint(10, 10, Math.toRadians(90), 1, thisIsABoolean);


    }
}