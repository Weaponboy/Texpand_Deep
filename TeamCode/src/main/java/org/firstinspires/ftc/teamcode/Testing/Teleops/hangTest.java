package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;

@TeleOp(name = "Hang_Test", group = "Testing")
public class hangTest extends OpMode {
    Servo hang1;
    Servo hang2;

    public ServoDegrees PTO = new ServoDegrees();

    MotorEx hangPower = new MotorEx();
    MotorEx hangPower2 = new MotorEx();
    MotorEx hangPower3 = new MotorEx();

    @Override
    public void init() {
        hang1=hardwareMap.get(Servo.class,"hang1");
        hang2=hardwareMap.get(Servo.class,"hang2");

        hangPower2.initMotor("slideMotor", hardwareMap);
        hangPower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangPower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangPower3.initMotor("slideMotor2", hardwareMap);
        hangPower3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangPower3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PTO.initServo("hangPTO", hardwareMap);
        PTO.setRange(new PwmControl.PwmRange(500, 2500), 270);
//        gripServo.setRange(180);
//        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);
//
        hangPower.initMotor("hangPower", hardwareMap);
        hangPower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PTO.setDirection(Servo.Direction.REVERSE);

        hang1.setPosition(0.5);
        hang2.setPosition(0.5);

        hangPower2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if (gamepad1.y){
            hang1.setPosition(0);
            hang2.setPosition(0);

        }else if (gamepad1.b){
            hang1.setPosition(1);
            hang2.setPosition(1);

        }else {
            hang1.setPosition(0.5);
            hang2.setPosition(0.5);
        }

        if (gamepad1.start){
            PTO.setPosition(135);
        }

        if (gamepad1.a){
            PTO.setPosition(100);
        }

        double power = 0;

        if (gamepad1.right_bumper){
            power = -1;
        } else if (gamepad1.left_bumper) {
            power = 1;
        }else {
            power = 0;
        }

        hangPower.update(power);
        hangPower2.update(power*0.5);
        hangPower3.update(power*0.5);

        telemetry.addData("rpm", hangPower.getVelocity());
        telemetry.update();

    }
}
