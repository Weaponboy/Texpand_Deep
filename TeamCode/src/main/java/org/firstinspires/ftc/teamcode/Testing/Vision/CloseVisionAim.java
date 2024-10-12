package org.firstinspires.ftc.teamcode.Testing.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.vision.Testing_SIM.SampleTargeting;
import dev.weaponboy.vision.SamplePipelines.singleSampleTargeting;

@TeleOp
public class CloseVisionAim extends OpModeEX {

    SampleTargeting sampleSorter = new SampleTargeting();

    singleSampleTargeting sampleTargeter = new singleSampleTargeting();
    VisionPortal portal;

    WebcamName webcam;

    @Override
    public void initEX() {
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcam);
        builder.addProcessor(sampleSorter);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(sampleSorter.getWidth(), sampleSorter.getHeight()));
        portal = builder.build();

        collection.camera.execute();

//        FtcDashboard.getInstance().startCameraStream(sampleSorter, 30);
    }

    @Override
    public void loopEX() {

        System.out.println("camera state: " + portal.getCameraState());

//        if (Math.abs(sampleSorter.getAngleRotate()) < 70){
//            collection.griperRotate.setPosition(90+sampleSorter.getAngleRotate());
//        }else {
//            collection.griperRotate.setPosition(90);
//        }
//
////
        if (currentGamepad1.a && !lastGamepad1.a){

            if (sampleSorter.getRailTarget() > 0 && sampleSorter.getRailTarget() < 20){
                collection.setRailTargetPosition(sampleSorter.getRailTarget());
            }

            double newSlidesTarget = collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35) + sampleSorter.getSlidesDelta();

            if (newSlidesTarget > 0 && newSlidesTarget < 35){
                if (sampleSorter.getSlidesDelta() > 0){
                    collection.queueCommand(collection.cameraSetPoint(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + sampleSorter.getSlidesDelta()+3)));
                }else {
                    collection.queueCommand(collection.cameraSetPoint(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + sampleSorter.getSlidesDelta()+3)));
                }
//                collection.queueCommand(collection.collect);
            }

            collection.griperRotate.setPosition(135+sampleSorter.getAngleRotate());
        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper){

            double railTarget = sampleSorter.getRailTarget()-10;
            railTarget = collection.getRailPosition()+railTarget;

            if (railTarget > 0 && railTarget < 20){
                collection.setRailTargetPosition(railTarget);
            }

            double newSlidesTarget = collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35) + sampleSorter.getSlidesDelta();

            if (newSlidesTarget > 0 && newSlidesTarget < 35){
                if (sampleSorter.getSlidesDelta() > 0){
                    collection.queueCommand(collection.collect(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + sampleSorter.getSlidesDelta())));
                }else {
                    collection.queueCommand(collection.collect(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + sampleSorter.getSlidesDelta())));
                }
//                collection.queueCommand(collection.collect);
            }

            collection.griperRotate.setPosition(135+sampleSorter.getAngleRotate());
        }


        if (gamepad1.right_bumper){
            collection.queueCommand(collection.collect(0));

        }

        if (currentGamepad1.x && !lastGamepad1.x){
            collection.setRailTargetPosition(10);
        }
//
        if (currentGamepad1.b && !lastGamepad1.b){
            collection.queueCommand(collection.collect);
        }

        if (currentGamepad1.y && !lastGamepad1.y){
            collection.camera.execute();
        }
//

        double newSlidesTarget = collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35) + sampleSorter.getSlidesDelta();
        telemetry.addData("horizontal", collection.horizontalMotor.getCurrentPosition()/(440/35));
        telemetry.addData("sampleSorter.getAngleRotate()", sampleSorter.getAngleRotate());
        telemetry.addData("getRailTarget", sampleSorter.getRailTarget());
        telemetry.addData("collection.horizontalMotor.getCurrentPosition()", sampleSorter.getSlidesDelta());
        telemetry.addData("newSlidesTarget", newSlidesTarget);
        telemetry.update();

    }

    @Override
    public void stop() {
        // Stop the vision processing pipeline
        if (sampleSorter != null) {
            portal.setProcessorEnabled(sampleSorter, false);
        }

        // Close the vision portal
        if (portal != null) {
            try {
                portal.close();
                portal = null;
            } catch (Exception e) {
                telemetry.addData("Error", "Error closing vision portal: " + e.getMessage());
            }
        }

        // Optionally reset USB device (if needed)
        try {
            webcam.close();
            webcam.resetDeviceConfigurationForOpMode();
        } catch (Exception e) {
            telemetry.addData("Error", "Error resetting USB: " + e.getMessage());
        }

        // Allow some time for everything to clean up
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        System.gc();

        telemetry.addData("Info", "Camera shut down successfully");
        telemetry.update();
    }

}
