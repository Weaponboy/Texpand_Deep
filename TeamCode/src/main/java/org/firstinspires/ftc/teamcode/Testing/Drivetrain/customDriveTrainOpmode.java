package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
