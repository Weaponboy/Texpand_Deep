package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.vision.Testing_SIM.UsingLineOfBestFir;

@TeleOp
public class NormalOpenCVTest extends OpModeEX {

    private OpenCvCamera webcam;
    private UsingLineOfBestFir pipeline;

    @Override
    public void initEX() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Initialize your pipeline
        pipeline = new UsingLineOfBestFir();
        webcam.setPipeline(pipeline);

        // Open the camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(680, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        collection.camera.execute();
    }

//    @Override
//    public void init_loop() {
//        if (currentGamepad1.x && !lastGamepad1.x && pipeline.isDisplayInput()){
//            pipeline.setDisplayInput(false);
//        }else if (currentGamepad1.x && !lastGamepad1.x && !pipeline.isDisplayInput()){
//            pipeline.setDisplayInput(true);
//        }
//        super.init_loop();
//    }

    @Override
    public void loopEX() {
        collection.queueCommand(collection.camera);
    }

//    @Override
//    public void stop() {
//        // Stop the webcam when the opmode is stopped
//        webcam.stopStreaming();
//    }
}
