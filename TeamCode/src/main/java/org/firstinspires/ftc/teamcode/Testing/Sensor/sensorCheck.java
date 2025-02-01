package org.firstinspires.ftc.teamcode.Testing.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import dev.weaponboy.command_library.Hardware.DistanceSensor;

@TeleOp
public class sensorCheck extends OpMode {

    public TouchSensor collectionBreakbeam;
    public TouchSensor depoTouch;

    public TouchSensor deliveryReset;
    public TouchSensor collectionReset;

    public DistanceSensor backLeft = new DistanceSensor();
    public DistanceSensor backRight = new DistanceSensor();
    public DistanceSensor right = new DistanceSensor();

    @Override
    public void init() {

        collectionBreakbeam = hardwareMap.get(TouchSensor.class, "clawsensor");
        depoTouch = hardwareMap.get(TouchSensor.class, "clawIR");
        deliveryReset = hardwareMap.get(TouchSensor.class, "DeliveryReset");
        collectionReset = hardwareMap.get(TouchSensor.class, "CollectionReset");

        backRight.init(hardwareMap, "backRight");
        backLeft.init(hardwareMap, "backLeft");
        right.init(hardwareMap, "right");

    }

    @Override
    public void loop() {

        telemetry.addData("intake Claw", collectionBreakbeam.isPressed());
        telemetry.addData("collection reset", collectionBreakbeam.isPressed());
        telemetry.addData("depo reset", deliveryReset.isPressed());
        telemetry.addData("depo claw sensor", depoTouch.isPressed());
        telemetry.addData("backLeft", backLeft.getPosition());
        telemetry.addData("backRight", backRight.getPosition());
        telemetry.addData("right", right.getPosition());
        telemetry.update();

    }
}
