package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

@TeleOp
public class OdometryTesting extends OpModeEX {

    @Override
    public void initEX() {


    }

    @Override
    public void loopEX() {
        odometry.queueCommand(odometry.updateLineBased);
        telemetry.addData("x", odometry.X());
        telemetry.addData("y",odometry.Y());
        telemetry.addData("heading", Math.toDegrees(odometry.Heading()));

        telemetry.addData("BackPod", odometry.currentBackPod);
        telemetry.addData("LeftPod", odometry.currentLeftPod);
        telemetry.addData("RightPod", odometry.currentRightPod);

        telemetry.addData("loopTime", loopTime);

        telemetry.update();

    }
}
