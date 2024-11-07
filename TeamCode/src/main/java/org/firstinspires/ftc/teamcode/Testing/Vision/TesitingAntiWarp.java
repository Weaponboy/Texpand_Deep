package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.FileWriter;
import java.io.IOException;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.vision.detectionData;

@TeleOp
public class TesitingAntiWarp extends OpModeEX {

    @Override
    public void initEX() {
        delivery.Scanning.execute();

        odometry.startPosition(79.5,100,0);

        FtcDashboard.getInstance().startCameraStream(collection.AntiWarp, 30);
    }

    @Override
    public void loopEX() {

//        if (currentGamepad1.b && !lastGamepad1.b && collection.getCurrentCommand() != collection.autoCollectGlobal){
//            collection.queueCommand(collection.autoCollectGlobal(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading())));
//        }else if (currentGamepad1.b && !lastGamepad1.b){
//            collection.overrideCurrent(true, collection.defaultCommand);
//        }
//
//        if (currentGamepad1.a && !lastGamepad1.a){
//            collection.queueCommand(collection.collect);
//        }
//
//        if (currentGamepad1.x && !lastGamepad1.x && collection.sampleSorterContour.isScanning() && !collection.sampleSorterContour.detections.isEmpty()){
//            collection.sampleSorterContour.setScanning(false);
//            collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()));
//            collection.angle = collection.sampleMap.get(0).getAngle();
//            //            collection.targetPointGlobal = collection.sampleSorterContour.convertToFieldCoor(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()));
//        }else if (currentGamepad1.x && !lastGamepad1.x && !collection.sampleSorterContour.isScanning()){
//            collection.sampleSorterContour.setScanning(true);
//        }
//
//        if (!collection.sampleSorterContour.isScanning()){
//            collection.portal.stopStreaming();
//        }else {
//            collection.portal.resumeStreaming();
//        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper){
            FileWriter fWriter = null;
            try {
                fWriter = new FileWriter("/sdcard/VisionLogs.txt", true);

                fWriter.write(System.lineSeparator());
                fWriter.write("WORKED!!!!");

                fWriter.flush();
            } catch (IOException e) {
                throw new RuntimeException(e);
            } finally {
                if (fWriter != null) {
                    try {
                        fWriter.close();
                    } catch (IOException e) {
                        e.printStackTrace();  // Handle any issues closing the writer
                    }
                }
            }
        }

//        telemetry.addData("slides position ", collection.horizontalMotor.getCurrentPosition()/(440/35));
//        telemetry.addData("rail position", collection.getRailPosition());
//        telemetry.addData("Loop time", loopTime);
//        telemetry.addData("X", odometry.X());
//        telemetry.addData("Y", odometry.Y());
//        telemetry.addData("Heading", odometry.Heading());
//        telemetry.addData("slides target ", collection.slideTargetPosition);
//        telemetry.addData("rail target", collection.railTarget);
//        telemetry.addData("Target point", collection.sampleMap.size());
//        for (detectionData detection: collection.sampleMap){
//            telemetry.addData("Target point", detection.getTargetPoint());
//        }
//        telemetry.addData("collection.sampleSorterContour.isScanning()", collection.sampleSorterContour.isScanning());
//        telemetry.addData("Rotate Angle", collection.angle);
//        telemetry.update();
    }

    @Override
    public void stop() {

    }

}
