package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestingLimelight extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double[] pythonOutput = result.getPythonOutput();
            telemetry.addData("got he", "got him!!!");
            if (pythonOutput != null && pythonOutput.length > 0) {
                int numPoints = (int) pythonOutput[0];
                telemetry.addData("Number of points", numPoints);

                for (int i = 0; i < numPoints && i < 3; i++) {
                    double x = pythonOutput[2*i + 1];
                    double y = pythonOutput[2*i + 2];
                    telemetry.addData("Point " + (i+1), String.format("(%.2f, %.2f)", x, y));
                }
            }
        }
        telemetry.update();

    }
}
