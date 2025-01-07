package org.firstinspires.ftc.teamcode.Teleop;

import android.companion.WifiDeviceFilter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;
import dev.weaponboy.vision.detectionData;

@TeleOp
public class sprint2TeleopSingle extends OpModeEX {
    boolean firstDrop = true;
    boolean rotated = false;
    boolean scanpos = false;
    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();

    double rotateTarget = 90;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();
    boolean queueCollection = false;

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;
    @Override
    public void initEX() {
        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        odometry.startPosition(1000,1000,0);

        collection.sampleSorterContour.setTargetColor(findAngleUsingContour.TargetColor.yellow);
        collection.sampleSorterContour.closestFirst = true;

//        collection.setCancelTransfer(false);
    }

    @Override
    public void loopEX() {


        // drive base code
        if(collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect){
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.5, (gamepad1.left_trigger - gamepad1.right_trigger)*0.4, -gamepad1.right_stick_x*0.5));

        }else {
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y * 0.9, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.65, -gamepad1.right_stick_x * 0.9));
        }
        /**
         * Overwrites
         * */
        if (currentGamepad1.back && !lastGamepad1.back){
            collection.sampleSorterContour.setScanning(false);
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
            scanpos = false;
            firstDrop = true;
        }
        /**
         * Collection code
         * */
        if (gamepad1.x && (collection.getFourBarState() == Collection.fourBar.transferUp || collection.getFourBarState() == Collection.fourBar.preCollect)){
            collection.setSlideTarget(45);
        }

        if ((currentGamepad1.left_stick_button && !(lastGamepad1.left_stick_button)) && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){

                if(collection.getFourBarState() == Collection.fourBar.preCollect) {
                    collection.queueCommand(collection.collect);
                    delivery.setGripperState(Delivery.gripper.drop);
                }

                if (collection.getChamberCollect()){
                    collection.queueCommand(collection.chamberCollect);
                }else if(collection.getFourBarState() == Collection.fourBar.stowedChamber){
                    collection.setClawsState(Collection.clawState.drop);
                }else {
                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(collection.transferDrop);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);
                }

                firstDrop = true;

        }

        if ((currentGamepad1.a && !lastGamepad1.a) || (currentGamepad1.right_bumper && !lastGamepad1.right_bumper)){
            collection.queueCommand(collection.collect);
            collection.setClawsState(Collection.clawState.drop);
        }

        if (currentGamepad1.left_stick_y < -0.5){
            collection.setSlideTarget(collection.getSlideTarget()+0.5);

//            if(collection.horizontalMotor.getCurrentPosition() > 40 && firstDrop && (collection.getFourBarState() != Collection.fourBar.collect || collection.getFourBarState() != Collection.fourBar.preCollect || collection.getFourBarState() != Collection.fourBar.collectChamber || collection.getFourBarState() != Collection.fourBar.preCollectFlipOut) && collection.getFourBarState() != Collection.fourBar.wallCollect){
//                collection.queueCommand(collection.collect);
//                delivery.setGripperState(Delivery.gripper.drop);
//                firstDrop = false;
//            }

        }else if (currentGamepad1.left_stick_y > 0.5){
            collection.setSlideTarget(collection.getSlideTarget()-0.5);
        }

        if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && collection.getChamberCollect()){
            collection.setChamberCollect(false);
            gamepad1.rumble(5);
        }else if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && !collection.getChamberCollect()){
            collection.setChamberCollect(true);
            gamepad1.rumble(5);
        }

        if (currentGamepad1.dpad_up && !lastGamepad1.dpad_up && collection.getFourBarState() != Collection.fourBar.wallCollect){

            collection.queueCommand(collection.wallCollect);
            delivery.setGripperState(Delivery.gripper.drop);

        } else if (currentGamepad1.dpad_up && !lastGamepad1.dpad_up && collection.getFourBarState() == Collection.fourBar.wallCollect) {
            collection.queueCommand(collection.wallTransfer);

            collection.queueCommand(collection.transferDrop);

            collection.queueCommand(delivery.closeGripper);

            collection.queueCommand(collection.openGripper);
        }

        if (currentGamepad1.left_stick_x < -0.3 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.wallCollect)){
            collection.setRailTargetPosition(collection.getRailPosition()-0.2);
        }else if (currentGamepad1.left_stick_x > 0.3 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.wallCollect)){
            collection.setRailTargetPosition(collection.getRailPosition()+0.2);
        }

        if (((gamepad1.dpad_left || (currentGamepad1.left_bumper && !lastGamepad1.left_bumper)) && !rotated) && collection.getFourBarState() == Collection.fourBar.preCollect){
            rotateTarget = 90;
            collection.griperRotate.setPosition(rotateTarget);
            rotated = true;
        }else if (((gamepad1.dpad_right || (currentGamepad1.left_bumper && !lastGamepad1.left_bumper)) && rotated) && collection.getFourBarState() == Collection.fourBar.preCollect){
            rotateTarget = 180;
            collection.griperRotate.setPosition(rotateTarget);
            rotated = false;
        }

        if(((currentGamepad1.b && !lastGamepad1.b) || (currentGamepad1.right_stick_button && !lastGamepad1.right_stick_button)) && !scanpos){
            delivery.queueCommand(delivery.cameraScan);
            scanpos = true;
        }else if (((currentGamepad1.b && !lastGamepad1.b) || (currentGamepad1.right_stick_button && !lastGamepad1.right_stick_button)) && scanpos){

            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

            collection.sampleSorterContour.setScanning(true);
            collection.portal.resumeStreaming();

            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;
            scanpos = false;
        }

//        if (currentGamepad1.y && !lastGamepad1.y) {
//            if (collection.sampleSorterContour.isScanning()){
//                raisingSlides = false;
//                collection.sampleSorterContour.setScanning(false);
//                collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(RobotPosition, delivery.getSlidePositionCM());
//            }
//
//            delivery.runReset();
//            delivery.behindNest.execute();
//            collection.queueCommand(collection.autoCollectGlobal);
//            collection.setChamberCollect(true);
//        }

        if (currentGamepad1.y && !lastGamepad1.y){

            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

            collection.sampleSorterContour.setScanning(true);
            collection.portal.resumeStreaming();

            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;

        }

        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else {
                collection.queueCommand(collection.transfer);

                collection.queueCommand(collection.transferDrop);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);
            }



            firstDrop = true;
            queueCollection = false;
        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;

            if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning()){

                collection.sampleSorterContour.setScanning(false);
                collection.portal.stopStreaming();
                collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

                collection.queueCommand(collection.autoCollectGlobal);
                collection.setChamberCollect(false);

                delivery.overrideCurrent(true, delivery.stow);
                delivery.runReset();

                queueCollection = true;
                busyDetecting = false;

                counter = 40;
            }


        } else if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter > 20) {

            collection.sampleSorterContour.setScanning(false);
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();

            busyDetecting = false;
        }

        if (busyDetecting && detectionTimer.milliseconds() > 500 && !collection.sampleSorterContour.detections.isEmpty()){
            busyDetecting = false;
            collection.sampleSorterContour.setScanning(false);
            collection.portal.stopStreaming();
            collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

            collection.queueCommand(collection.autoCollectGlobal);
            collection.setChamberCollect(false);

            delivery.overrideCurrent(true, delivery.stow);
            delivery.runReset();
        }

        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            collection.queueCommand(collection.transfer);

            collection.queueCommand(collection.transferDrop);

            collection.queueCommand(delivery.closeGripper);

            collection.queueCommand(collection.openGripper);

            queueCollection = false;
        }

        /**
         * Delivery code
         * */
        if (currentGamepad1.start && !lastGamepad1.start && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed()){
            delivery.queueCommand(delivery.preClipBack);
        }else if (currentGamepad1.start && !lastGamepad1.start && delivery.slideMotor.getCurrentPosition() > 100){
            delivery.queueCommand(delivery.clipBack);
        }

//        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer){
//
//            collection.queueCommand(collection.transferDrop);
//
//            collection.queueCommand(delivery.closeGripper);
//
//            collection.queueCommand(collection.openGripper);
//
//        }
        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.slideMotor.getCurrentPosition() < 700 && !(collection.getFourBarState()== Collection.fourBar.preCollect)){

            delivery.slideSetPoint(delivery.highBasket);
            delivery.slides = Delivery.slideState.moving;

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.slideMotor.getCurrentPosition() > 700 && !(collection.getFourBarState()== Collection.fourBar.preCollect)){
            delivery.queueCommand(delivery.deposit);
        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("rail position ", collection.getRailPosition());
        telemetry.addData("horizontal slides ", collection.horizontalMotor.getCurrentPosition());
        telemetry.addData("vertical slides ", delivery.getSlidePositionCM());
        telemetry.addData("collection current command ", collection.getCurrentCommand());
        telemetry.addData("delivery slides", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor collection", collection.clawSensor.isPressed());
        telemetry.addData("claw sensor delivery", delivery.clawSensor.isPressed());
        telemetry.addData("slide target", collection.getSlideTarget());
        telemetry.addData("Target point", collection.sampleMap.size());
        for (detectionData detection: collection.sampleMap){
            telemetry.addData("Target point", detection.getTargetPoint());
        }
        telemetry.update();
    }
}