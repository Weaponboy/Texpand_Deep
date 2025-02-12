package dev.weaponboy.vision.Archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class Get_Encoder_Positions extends OpMode {

    DcMotor arm_Motor;
    DcMotor wrist_Motor;

    @Override
    public void init() {

        arm_Motor = hardwareMap.get(DcMotor.class,"arm_Motor");
        wrist_Motor = hardwareMap.get(DcMotor.class,"wrist_Motor");

        arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrist_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        if (gamepad1.a){
            arm_Motor.setPower(0.6);
        }

        telemetry.addData("Arm position", arm_Motor.getCurrentPosition());
        telemetry.addData("Wrist position", wrist_Motor.getCurrentPosition());
        telemetry.update();
    }
}
