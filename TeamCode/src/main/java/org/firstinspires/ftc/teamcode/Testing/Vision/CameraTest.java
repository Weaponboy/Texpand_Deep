package org.firstinspires.ftc.teamcode.Testing.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;
import dev.weaponboy.vision.SamplePipelines.testingCamera;

@TeleOp
public class CameraTest extends OpMode {

    public testingCamera sampleSorterContour = new testingCamera();
    public VisionPortal portal;

    @Override
    public void init() {
        WebcamName closeAim = hardwareMap.get(WebcamName.class, "webcam");
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(closeAim);
        builder.addProcessor(sampleSorterContour);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(1280, 960));
        portal = builder.build();

//        FtcDashboard.getInstance().startCameraStream(sampleSorterContour, 30);
    }

    @Override
    public void loop() {

    }
}
