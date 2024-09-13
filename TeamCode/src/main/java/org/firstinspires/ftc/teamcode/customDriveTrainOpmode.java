package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

@TeleOp
public class customDriveTrainOpmode extends OpModeEX {
    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
        driveBase.driveFieldCentric(-gamepad1.right_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);

    }
}
