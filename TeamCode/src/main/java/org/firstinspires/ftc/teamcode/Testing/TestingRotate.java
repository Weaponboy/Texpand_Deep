package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import dev.weaponboy.command_library.Hardware.ServoDegrees;

@TeleOp
public class TestingRotate extends OpMode {

    public ServoDegrees griperRotate= new ServoDegrees();

    Servo Test;

    @Override
    public void init() {
//        griperRotate.initServo("gripperRotate", hardwareMap);

        Test = hardwareMap.get(Servo.class, "gripperRotate");

//        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 270);
//
//        griperRotate.setPosition(135);

        Test.setPosition(0.5);
    }

    @Override
    public void loop() {

//        if (gamepad1.back){
//            griperRotate.setPosition(griperRotate.getPositionDegrees() + 1);
//        } else if (gamepad1.start) {
//            griperRotate.setPosition(griperRotate.getPositionDegrees() - 1);
//        }
//
//        telemetry.addData("position", griperRotate.getPositionDegrees());
//        telemetry.update();
    }
}
