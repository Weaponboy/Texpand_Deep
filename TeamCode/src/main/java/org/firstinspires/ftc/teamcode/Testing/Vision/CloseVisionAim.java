package org.firstinspires.ftc.teamcode.Testing.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.vision.Testing_SIM.SampleTargeting;
import dev.weaponboy.vision.SamplePipelines.singleSampleTargeting;

@TeleOp
public class CloseVisionAim extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();
    SampleTargeting sampleSorter = new SampleTargeting();

    singleSampleTargeting sampleTargeter = new singleSampleTargeting(dashboardTelemetry);
    VisionPortal portal;

    @Override
    public void init() {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        builder.addProcessor(sampleSorter);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(1920, 1080));
        portal = builder.build();

//        FtcDashboard.getInstance().startCameraStream(sampleSorter, 30);
    }

    @Override
    public void loop() {

    }
}
