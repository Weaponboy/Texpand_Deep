package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Limelight;

@TeleOp(name = "A_Limelight_Testing", group = "Testing")
public class TestingLimelight extends OpModeEX {

    @Override
    public void initEX() {

        odometry.startPosition(100, 100, 0);

        limelight.collectColoredSamples(true);
        limelight.setDetectionColor(true);
        limelight.setAuto(true);

        limelight.setTargetColor(Limelight.color.yellow);

        limelight.setReturningData(true);
        limelight.setGettingResults(true);

        collection.disableServos();
        delivery.disableServos();

    }

    @Override
    public void loopEX() {

        if (gamepad1.b){
            limelight.switchPipeline(7);
            limelight.setReturningData(true);
            limelight.setGettingResults(true);
        }

        if (gamepad1.dpad_right){
            limelight.collectColoredSamples(true);
        }

        if (gamepad1.dpad_left){
            limelight.collectColoredSamples(false);
        }

        if (gamepad1.a){
            limelight.setDetectionColor(false);
        }

        if (gamepad1.y){
            limelight.setDetectionColor(true);
        }

        telemetry.addData("Toggle collection coloured samples", "a = false, y = true");
        telemetry.addData("Vision targeting coloured samples", limelight.getAllianceSamples());

        telemetry.addData("Set alliance color", "d_pad_right = Red, d_pad_left = Blue");
        telemetry.addData("targeting red samples", limelight.isAllianceColor());

        telemetry.addData("returning results", limelight.returningData);
        telemetry.addData("getting results ", limelight.isGettingResults);
        telemetry.addData("limelight results", limelight.getTargetPoint());
        if (limelight.getTargetPoint() != null){
            telemetry.addData("limelight results X", limelight.getTargetPoint().getTargetPoint().getX());
            telemetry.addData("limelight results Y", limelight.getTargetPoint().getTargetPoint().getY());
            telemetry.addData("limelight results angle", limelight.getTargetPoint().getAngle());
        }
        telemetry.update();
    }
}
