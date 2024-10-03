package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
@TeleOp
public class hangTest extends OpMode {
    CRServo hang1;
    CRServo hang2;
    @Override
    public void init() {
        hang1=hardwareMap.get(CRServo.class,"hang1");
        hang2=hardwareMap.get(CRServo.class,"hang2");

    }

    @Override
    public void loop() {
        if (gamepad1.y){
        hang1.setPower(0);
        hang2.setPower(0);

        }else {
            hang1.setPower(0.5);
            hang2.setPower(0.5);
        }
    }
}
