package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.weaponboy.command_library.Hardware.MotorEx;

@TeleOp(name = "untangleProgram", group = "AAAAAAt the top")
public class untangleProgram extends OpMode {

    MotorEx LF = new MotorEx();
    MotorEx RF = new MotorEx();
    MotorEx RB = new MotorEx();
    MotorEx LB = new MotorEx();

    MotorEx rightSlide = new MotorEx();
    MotorEx leftSlide = new MotorEx();

    @Override
    public void init() {
        LF.initMotor("LF", hardwareMap);
        RF.initMotor("RF", hardwareMap);
        LB.initMotor("LB", hardwareMap);
        RB.initMotor("RB", hardwareMap);

        rightSlide.initMotor("slideMotor", hardwareMap);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.initMotor("slideMotor2", hardwareMap);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        drive(gamepad1.right_stick_y*0.5, (gamepad1.left_trigger - gamepad1.right_trigger)*0.4, -gamepad1.right_stick_x*0.5);

        if (gamepad1.left_stick_x > 0){
            rightSlide.update(-0.5);
            leftSlide.update(-0.5);
        }else if (gamepad1.left_stick_x < 0){
            rightSlide.update(0.5);
            leftSlide.update(0.5);
        }else {
            rightSlide.update(0);
            leftSlide.update(0);
        }

    }

    public void drive(double vertikal, double turn, double strafe){

        double denominator = Math.max(1, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));

        LF.update((vertikal-strafe-turn)/denominator);
        RF.update((vertikal+strafe+turn)/denominator);
        LB.update((vertikal+strafe-turn)/denominator);
        RB.update((vertikal-strafe+turn)/denominator);
    }
}
