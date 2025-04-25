package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

@TeleOp(name = "Motor_Test", group = "Testing")
public class MotorTest extends OpMode {

    DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "LB");
    }

    @Override
    public void loop() {
        motor.setPower(1);

        System.out.println(motor.getVelocity());
        System.out.println(motor.getCurrent(CurrentUnit.MILLIAMPS));
    }

}
