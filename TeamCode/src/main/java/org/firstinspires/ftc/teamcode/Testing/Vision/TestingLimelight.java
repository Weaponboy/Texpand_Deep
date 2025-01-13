package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestingLimelight extends LinearOpMode {

    private Limelight3A limelight;

    int counter = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);

        waitForStart();

        limelight.start();

        while (opModeIsActive()){

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                telemetry.addData("result", result.getPythonOutput()[0]);
            }

            telemetry.addData("result", result);
            telemetry.addData("limelight.isRunning();", limelight.isConnected());
            telemetry.addData("counter", counter);
            telemetry.update();

//            telemetry.addData("result", result.getPythonOutput()[0]);

//            if(counter > 1500){
//                counter = 0;
//                limelight.start();
//            }else if (counter > 1000){4
//                limelight.stop();
//            }

        }

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
