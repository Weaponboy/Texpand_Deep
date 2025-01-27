package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;
import dev.weaponboy.vision.detectionData;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

@TeleOp
public class globalVisionTesting extends OpModeEX {

    boolean queuedClipCommands = false;
//
    boolean runningClipAndCollect = false;
    boolean detectingInSub = true;
    ElapsedTime detectingTimer = new ElapsedTime();

    boolean raisingSlides = false;

    enum Obs_Collect{
        waiting,
        extendSlides,
        flipArm,
        drop
    }

    Obs_Collect obsCollect = Obs_Collect.waiting;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void initEX() {
//        delivery.PreClip.execute();

        odometry.startPosition(82.5,100,0);

//        odometry.startPosition(100,100,0);

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        collection.gripServo.setPosition(120);

        delivery.setGripperState(Delivery.gripper.grab);

        collection.sampleSorterContour.closestFirst = true;

//        collection.sampleSorterContour.setScanning(false);

        collection.sampleSorterContour.setTargetColor(findAngleUsingContour.TargetColor.yellow);
    }

    @Override
    public void loopEX() {

        if(currentGamepad1.b && !lastGamepad1.b){
            delivery.queueCommand(delivery.cameraScan);
        }

//        if (gamepad1.dpad_up){
//            collection.setSlideTarget(10);
//            collection.setChamberCollect(true);
//        } else if (gamepad1.dpad_down) {
//            collection.setSlideTarget(0);
//            collection.setChamberCollect(false);
//        }
//
//        if (currentGamepad1.left_trigger > 0 && !(lastGamepad1.left_trigger > 0)){
//            collection.sampleSorterContour.setScanning(true);
//            collection.portal.resumeStreaming();
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//
//            collection.sampleSorterContour.setScanning(false);
//            collection.portal.stopStreaming();
//            collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));
//
//            collection.queueCommand(collection.autoCollectChamber);
////            delivery.fourbarState = Delivery.fourBarState.postTransfer;
//
//
////            delivery.queueCommand(delivery.preClip);
//            delivery.queueCommand(delivery.clipFront);
//            collection.setChamberCollect(true);
//        }

//        if (gamepad1.back){
//            collection.sampleSorterContour.setScanning(false);
//            delivery.overrideCurrent(true, delivery.stow);
//            collection.overrideCurrent(true, collection.stow);
//            delivery.runReset();
//        }
//
//        if (currentGamepad1.a && !lastGamepad1.a){
//            collection.queueCommand(collection.collect);
//        }
//

//        if (gamepad1.left_bumper){
//            delivery.slideSetPoint(delivery.highChamberFront);
//            delivery.slides = Delivery.slideState.moving;
//
//        }
//
//        if (gamepad1.start){
//            runningClipAndCollect = true;
//            queuedClipCommands = false;
//            detectingInSub = true;
//        }

        if (gamepad1.a && !lastGamepad1.a){
            collection.queueCommand(collection.autoCollectGlobal);
            collection.setChamberCollect(false);

            delivery.overrideCurrent(true, delivery.stow);
            delivery.runReset();
        }

        if (currentGamepad1.x && !lastGamepad1.x && collection.sampleSorterContour.isScanning()){
            collection.sampleSorterContour.setScanning(false);
            collection.portal.stopStreaming();
        }

        if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning()){
            collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));
        }

        if (currentGamepad1.x && !lastGamepad1.x && !collection.sampleSorterContour.isScanning()){
            collection.sampleSorterContour.setScanning(true);
            collection.portal.resumeStreaming();
        }

        if (!collection.sampleSorterContour.isScanning() || delivery.getSlidePositionCM() < 15){

        }else {
//            collection.portal.resumeStreaming();
            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
//            delivery.secondPivot.setPosition(80);
//            delivery.fourbarState = Delivery.fourBarState.basketDeposit;
//            delivery.mainPivot.setPosition(190.5);

            //0.794
            //1.2587

            //190.5

        }

//        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper){
//            FileWriter fWriter = null;
//            try {
//                fWriter = new FileWriter("/sdcard/VisionLogs.txt", true);
//
//                fWriter.write(System.lineSeparator());
//                fWriter.write("WORKED!!!!");
//
//                fWriter.flush();
//            } catch (IOException e) {
//                throw new RuntimeException(e);
//            } finally {
//                if (fWriter != null) {
//                    try {
//                        fWriter.close();
//                    } catch (IOException e) {
//                        e.printStackTrace();  // Handle any issues closing the writer
//                    }
//                }
//            }
//        }
//
//        if (runningClipAndCollect){
//            clipAndCollect();
//        }
//
//        if (gamepad1.right_trigger > 0){
//            obsCollect = Obs_Collect.extendSlides;
//        } else if (obsCollect != Obs_Collect.waiting) {
//            ObsCollecting();
//        }

//        if (gamepad1.right_stick_y < -0.5 && slideMotor.getCurrentPosition() < 10){
//            slideMotor.setPower(0.5);
//        }else if (gamepad1.right_stick_y > 0.5 && slideMotor.getCurrentPosition() > 10){
//            slideMotor.setPower(-0.5);
//        }else {
//            slideMotor.setPower(0);
//        }



//        double ticksPerCM = (double) 1050 / 71;

        telemetry.addData("slides position ", obsCollect);
        telemetry.addData("collection slides position ", collection.getSlidePositionCM());
        telemetry.addData("delivery position ", delivery.getSlidePositionCM());
//        telemetry.addData("delivery arm ", delivery.findCameraScanPosition());
//        telemetry.addData("delivery arm actual", delivery.mainPivot.getPositionDegrees());

        //        telemetry.addData("rail position", collection.getRailPosition());
        telemetry.addData("Loop time", loopTime);
        telemetry.addData("X", odometry.X());
        telemetry.addData("Y", odometry.Y());
        telemetry.addData("Heading", odometry.Heading());
        telemetry.addData("slides target ", collection.slideTargetPosition);
        telemetry.addData("rail target", collection.railTarget);
        telemetry.addData("Target point", collection.sampleMap.size());
        telemetry.addData("Arm angle", 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));
        for (detectionData detection: collection.sampleMap){
            telemetry.addData("Target point", detection.getTargetPoint());
        }
        telemetry.addData("collection.sampleSorterContour.isScanning()", collection.sampleSorterContour.isScanning());
        telemetry.addData("Rotate Angle", collection.angle);
        telemetry.update();
    }

    public void ObsCollecting(){

        if (obsCollect == Obs_Collect.extendSlides && collection.getSlideTarget() < 10){
            collection.setSlideTarget(34.5);
        }else if (obsCollect == Obs_Collect.extendSlides && collection.getSlidePositionCM() > 34){
            obsCollect = Obs_Collect.flipArm;
            collection.setRailTargetPosition(18);
            collection.ChamberCollect.execute();
            timer.reset();
        }else if (obsCollect == Obs_Collect.flipArm && timer.milliseconds() > 550 && timer.milliseconds() < 700){
            collection.setClawsState(Collection.clawState.drop);
            collection.setRailTargetPosition(13);
            timer.reset();
            obsCollect = Obs_Collect.drop;
        }else if (obsCollect == Obs_Collect.drop && timer.milliseconds() > 700) {
            obsCollect = Obs_Collect.waiting;
            collection.queueCommand(collection.collect);
            collection.queueCommand(collection.collect);
        }
    }

    public void clipAndCollect(){

        if (detectingInSub && delivery.getCurrentCommand() != delivery.clipFront){
            detectingInSub = false;
            detectingTimer.reset();
            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
        }

        if (!queuedClipCommands && !detectingInSub && detectingTimer.milliseconds() > 500) {

            if (!collection.sampleSorterContour.detections.isEmpty()){
                collection.sampleSorterContour.setScanning(false);
                collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(RobotPosition, delivery.getSlidePositionCM());
                collection.setSlideTarget(20);
                collection.queueCommand(collection.slidesTargeting);
                collection.queueCommand(collection.autoCollectGlobal);
                collection.setChamberCollect(true);
            }else {
                for (int i = 0; i < 10; i++){
                    try {
                        Thread.sleep(5);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    if (!collection.sampleSorterContour.detections.isEmpty()){
                        collection.sampleSorterContour.setScanning(false);
                        collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(RobotPosition, delivery.getSlidePositionCM());
                        collection.setSlideTarget(20);
                        collection.queueCommand(collection.slidesTargeting);
                        collection.queueCommand(collection.autoCollectGlobal);
                        collection.setChamberCollect(true);
                    }
                }
            }

            queuedClipCommands = true;

            delivery.queueCommand(delivery.clipFront);
            delivery.queueCommand(delivery.clipFront);
            delivery.queueCommand(delivery.clipFront);

            runningClipAndCollect = false;

        }

    }

    @Override
    public void stop() {

    }

}
