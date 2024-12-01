//package org.firstinspires.ftc.teamcode.Testing.Vision;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
//import dev.weaponboy.vision.Testing_SIM.MatImagePipeline;
//import dev.weaponboy.vision.Testing_SIM.UsingLineOfBestFir;
//
//@TeleOp
//public class MatImageTesting extends OpModeEX {
//
//    private OpenCvCamera webcam;
//    private MatImagePipeline pipeline;
//    @Override
//    public void initEX() {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        // Initialize your pipeline
//        pipeline = new MatImagePipeline();
//        webcam.setPipeline(pipeline);
//
//        // Open the camera
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error", errorCode);
//                telemetry.update();
//            }
//        });
//
//        collection.camera.execute();
//    }
//
//    @Override
//    public void loopEX() {
//        collection.queueCommand(collection.camera);
//    }
//
//    @Override
//    public void stop() {
//        // Stop the webcam when the opmode is stopped
//        webcam.stopStreaming();
//    }
//}
