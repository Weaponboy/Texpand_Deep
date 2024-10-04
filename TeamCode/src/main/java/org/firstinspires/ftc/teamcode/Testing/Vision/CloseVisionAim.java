package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.vision.SamplePipelines.singleSampleTargeting;
import dev.weaponboy.vision.Testing_SIM.testingSortingSamples;

@TeleOp
public class CloseVisionAim extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();
    testingSortingSamples sampleSorter = new testingSortingSamples();

    singleSampleTargeting sampleTargeter = new singleSampleTargeting(dashboardTelemetry);
    VisionPortal portal;

    @Override
    public void init() {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        builder.addProcessor(sampleSorter);
        portal = builder.build();

//        FtcDashboard.getInstance().startCameraStream(sampleSorter, 30);
    }

    @Override
    public void loop() {

    }
}
