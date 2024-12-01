package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
@TeleOp
public class hangTest extends OpMode {
    Servo hang1;
    Servo hang2;
    @Override
    public void init() {
        hang1=hardwareMap.get(Servo.class,"hang1");
        hang2=hardwareMap.get(Servo.class,"hang2");

        hang1.setPosition(0.5);  // ensure power is explicitly set to 0 after initialization
        hang2.setPosition(0.5);

//        hang1.setDirection(CRServo.Direction.REVERSE);

        try {
            Thread.sleep(100);  // pause to stabilize initialization
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        hang1.setPosition(0.5);  // ensure power is explicitly set to 0 after initialization
        hang2.setPosition(0.5);  // ensure power is explicitly set to 0 after initialization

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
//        telemetry.addData("pos", hang1.getPosition());

    }
}
