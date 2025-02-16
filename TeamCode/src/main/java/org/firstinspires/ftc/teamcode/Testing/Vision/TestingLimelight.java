package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

@TeleOp
@Disabled
public class TestingLimelight extends OpModeEX {

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
        telemetry.addData("limelight results", limelight.getTargetPoint());
        telemetry.update();
    }

//    @Override
//    public void loop() {
//
//
////        if (result != null) {
////            double[] pythonOutput = result.getPythonOutput();
////            telemetry.addData("got he", "got him!!!");
////            if (pythonOutput != null && pythonOutput.length > 0) {
////                int numPoints = (int) pythonOutput[0];
////                telemetry.addData("Number of points", numPoints);
////
////                for (int i = 0; i < numPoints && i < 3; i++) {
////                    double x = pythonOutput[2*i + 1];
////                    double y = pythonOutput[2*i + 2];
////                    telemetry.addData("Point " + (i+1), String.format("(%.2f, %.2f)", x, y));
////                }
////            }
////        }
//
//
//
//    }
}
