package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.HardwareQueue;
import org.firstinspires.ftc.teamcode.hardware.PriorityMotor;

public class Sensors {
    private LynxModule controlHub, expansionHub;
    private final HardwareQueue hardwareQueue;
    private final HardwareMap hardwareMap;
    private Robot robot;

    //private IMU imu;
    private int[] odometry = new int[]{0, 0, 0};

    private int slidesEncoder;
    private double slidesVelocity;
    private boolean slidesDown = false;
    private boolean intakeTriggered = false;
    private boolean depositTouched = false;

    private final AnalogInput[] analogEncoders = new AnalogInput[2];
    private final AnalogInput backUltrasonic, frontUltrasonic;
    private double backUltrasonicDist, frontUltrasonicDist = 0;
    public double[] analogVoltages = new double[analogEncoders.length];

    private double voltage;

    private DigitalChannel depositLimitSwitch;

    HuskyLens.Block[] huskyLensBlocks;

    private double leftFrontMotorCurrent, leftRearMotorCurrent, rightRearMotorCurrent, rightFrontMotorCurrent;

    public static double voltageK = 0.3;

    public Sensors(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = hardwareQueue;
        this.robot = robot;

        initSensors(hardwareMap);
    }

    private void initSensors(HardwareMap hardwareMap) {
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        depositLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update() {
        updateControlHub();
        updateExpansionHub();
        updateTelemetry();
    }


    private double imuUpdateTime = 15;
    public double timeTillNextIMUUpdate = imuUpdateTime;
    public boolean imuJustUpdated = false;

    private double voltageUpdateTime = 5000;
    long lastVoltageUpdatedTime = System.currentTimeMillis();

    private double huskyUpdateTime = 100;
    long lastHuskyLensUpdatedTime = System.currentTimeMillis();
    public boolean huskyJustUpdated = false;

    private void updateControlHub() {
        odometry[0] = ((PriorityMotor) hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition(); // left (0)
        odometry[1] = ((PriorityMotor) hardwareQueue.getDevice("rightBack")).motor[0].getCurrentPosition(); // back (2)
        odometry[2] = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition(); // right (3)

        long currTime = System.currentTimeMillis();


        if (currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = currTime;
        }

        slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition() * -1;
        slidesVelocity = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getVelocity() * -1;

        depositTouched = !depositLimitSwitch.getState();

        backUltrasonicDist = backUltrasonic.getVoltage() / 3.2 * 1000;
        frontUltrasonicDist = frontUltrasonic.getVoltage() / 3.2 * 1000;
    }

    private void updateExpansionHub() {
        try {
        } catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "expansion hub failed");
        }
    }


    public int[] getOdometry() {
        return odometry;
    }

    public double getVoltage() {
        return voltage;
    }

    public void updateDrivetrainMotorCurrents() {
        leftFrontMotorCurrent = robot.drivetrain.leftFront.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        leftRearMotorCurrent = robot.drivetrain.leftBack.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        rightRearMotorCurrent = robot.drivetrain.rightBack.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        rightFrontMotorCurrent = robot.drivetrain.rightFront.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
    }


}
