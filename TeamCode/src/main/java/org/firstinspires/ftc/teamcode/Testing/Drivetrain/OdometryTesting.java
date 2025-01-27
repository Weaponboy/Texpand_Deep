package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

@TeleOp
public class OdometryTesting extends OpModeEX {

    @Override
    public void initEX() {

//        odometry.startPosition(342.5, 164, 180);

        odometry.startPosition(0, 0, 0);
    }

    @Override
    public void loopEX() {
        odometry.queueCommand(odometry.updateLineBased);

        telemetry.addData("x", odometry.X());
        telemetry.addData("y",odometry.Y());
        telemetry.addData("heading", odometry.Heading());

        telemetry.addData("BackPod", odometry.currentBackPod);
        telemetry.addData("LeftPod", odometry.currentLeftPod);
        telemetry.addData("RightPod", odometry.currentRightPod);

        telemetry.addData("loopTime", loopTime);

        telemetry.update();

    }
}
