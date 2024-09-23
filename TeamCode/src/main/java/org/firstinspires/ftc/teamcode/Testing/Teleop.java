package org.firstinspires.ftc.teamcode.Testing;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

public class Teleop extends OpModeEX {

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
        driveBase.drivePowers(gamepad1.right_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
        if (gamepad1.right_bumper) {
            driveBase.drivePowers(gamepad1.right_stick_y,gamepad1.right_stick_x,driveBase.headindinglockMotorPower(odometry.headingError(90)));
        }
    }
}
