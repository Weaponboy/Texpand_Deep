package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

@TeleOp(name = "Testing_Vision_Time_Reset", group = "Testing")
public class TestingVisiontime extends OpModeEX {

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
//        System.out.println("Limelight time" + limelight.getCurrentTime());
//        System.out.println("Robot time" + limelight.getRobotTime());

        telemetry.addData("Limelight time", limelight.getCurrentTime());
        telemetry.addData("Robot time", limelight.getRobotTime());
        telemetry.update();
    }

}
