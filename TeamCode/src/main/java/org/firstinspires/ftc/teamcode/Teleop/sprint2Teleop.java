package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;
import dev.weaponboy.vision.detectionData;

@TeleOp
public class sprint2Teleop extends OpModeEX {

    boolean cameraScan = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    boolean fastTransfer = true;
    boolean queueCollection = false;

    boolean autoPreClip = false;
    boolean ranPreClip = false;

    @Override
    public void initEX() {
        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        odometry.startPosition(1000,1000,0);

        collection.sampleSorterContour.setTargetColor(findAngleUsingContour.TargetColor.yellow);
        collection.sampleSorterContour.closestFirst = true;
    }

    @Override
    public void loopEX() {

        // drive base code
        if(collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect){
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.5, (gamepad1.left_trigger - gamepad1.right_trigger)*0.4, -gamepad1.right_stick_x*0.5));
        }else {
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y * 0.9, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.6, -gamepad1.right_stick_x * 0.9));
        }

        /**
         * Overwrites
         * */
        if (currentGamepad2.back && !lastGamepad2.back){
            collection.sampleSorterContour.setScanning(false);
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
        }

        /**
         * Collection code
         * */
        if (gamepad2.x && (collection.getFourBarState() == Collection.fourBar.transferUp || collection.getFourBarState() == Collection.fourBar.preCollect)){
            collection.setSlideTarget(50);
        }

        if (currentGamepad2.right_bumper && !lastGamepad2.right_bumper){

            delivery.setGripperState(Delivery.gripper.drop);
            delivery.overrideCurrent(true, delivery.stow);
            delivery.griperRotateSev.setPosition(0);

            collection.queueCommand(collection.collect);
        }

        if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up && fastTransfer){
            fastTransfer = false;
        }else if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up && !fastTransfer){
            fastTransfer = true;
        }

        if (currentGamepad2.dpad_down && !lastGamepad2.dpad_down && collection.getChamberCollect()){
            collection.setChamberCollect(false);
            gamepad2.rumble(100);
        }else if (currentGamepad2.dpad_down && !lastGamepad2.dpad_down && !collection.getChamberCollect()){
            collection.setChamberCollect(true);
            gamepad2.rumble(100);
        }

        if (currentGamepad2.right_trigger > 0 && !(lastGamepad2.right_trigger > 0) && collection.getFourBarState() == Collection.fourBar.collect){

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else {
                if (fastTransfer){
                    collection.queueCommand(collection.transferSlam);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDropSlam);
                }else {
                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDrop);
                }

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);
            }
        }

        if (currentGamepad2.left_stick_y < -0.4){
            collection.setSlideTarget(collection.getSlideTarget()+Math.abs(currentGamepad2.left_stick_y));
        }

        if (currentGamepad2.left_stick_y > 0.4){
            collection.setSlideTarget(collection.getSlideTarget()-Math.abs(currentGamepad2.left_stick_y));
        }

        if (currentGamepad2.left_stick_x < -0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.setRailTargetPosition(collection.getRailPosition()-Math.abs(currentGamepad2.left_stick_x*0.5));
        }else if (currentGamepad2.left_stick_x > 0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.setRailTargetPosition(collection.getRailPosition()+Math.abs(currentGamepad2.left_stick_x*0.5));
        }

        if (currentGamepad2.left_trigger > 0.1 && lastGamepad2.left_trigger < 0.1 && collection.griperRotate.getPositionDegrees() > 110 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.griperRotate.setPosition(90);
        }else if (currentGamepad2.left_trigger > 0.1 && lastGamepad2.left_trigger < 0.1 && collection.griperRotate.getPositionDegrees() < 110 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.griperRotate.setPosition(180);
        }

        if (gamepad2.start){
            hang.hang1.setPosition(1);
            hang.hang2.setPosition(1);
        }else if (gamepad1.dpad_right){
            hang.hang1.setPosition(0);
            hang.hang2.setPosition(0);
        }else{
            hang.hang1.setPosition(0.5);
            hang.hang2.setPosition(0.5);
        }

        if(currentGamepad2.b && !lastGamepad2.b){
            delivery.queueCommand(delivery.cameraScan);
            cameraScan = true;
        }

        if (cameraScan && delivery.getSlidePositionCM() > 15){
            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
        }

        if (currentGamepad2.a && !lastGamepad2.a){
            collection.setClawsState(Collection.clawState.drop);
        }

        if (currentGamepad2.y && !lastGamepad2.y && delivery.getSlidePositionCM() > 15){

            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

            collection.sampleSorterContour.setScanning(true);
//            collection.portal.resumeStreaming();

            cameraScan = false;
            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;

        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;

            if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning()){

                collection.sampleSorterContour.setScanning(false);
//                collection.portal.stopStreaming();
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
//            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();

            busyDetecting = false;
        }

        if (collection.sampleSorterContour.isScanning() && !collection.sampleSorterContour.detections.isEmpty() && delivery.getSlidePositionCM() > 10 && odometry.getXVelocity() < 10){
            gamepad2.rumble(5);
        }

        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else {
                if (fastTransfer){
                    collection.queueCommand(collection.transferSlam);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDropSlam);
                }else {
                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDrop);
                }

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);
            }

            queueCollection = false;
        }

        if (currentGamepad1.start && lastGamepad1.start && autoPreClip){
            autoPreClip = false;
            gamepad1.rumble(300);
        }else if (currentGamepad1.start && lastGamepad1.start && !autoPreClip){
            autoPreClip = true;
            gamepad1.rumble(300);
        }

        /**
         * Delivery code
         * */
        if (!ranPreClip && autoPreClip && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed() && collection.getCurrentCommand() == collection.defaultCommand){
            delivery.queueCommand(delivery.preClipFront);
            delivery.griperRotateSev.setPosition(90);

            ranPreClip = true;
        }

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed() && collection.getCurrentCommand() == collection.defaultCommand){
            delivery.queueCommand(delivery.preClipFront);
            delivery.griperRotateSev.setPosition(90);
        }else if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() > 100){
            delivery.queueCommand(delivery.clipFront);
            ranPreClip = false;
        }

        if ((currentGamepad2.left_stick_button && !(lastGamepad2.left_stick_button)) && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){

            if(collection.getFourBarState() == Collection.fourBar.preCollect) {
                collection.queueCommand(collection.collect);
                delivery.setGripperState(Delivery.gripper.drop);
            }

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else if(collection.getFourBarState() == Collection.fourBar.stowedChamber){
                collection.setClawsState(Collection.clawState.drop);
            }else {
                if (fastTransfer){
                    collection.queueCommand(collection.transferSlam);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDropSlam);
                }else {
                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDrop);
                }

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);
            }

        }

        if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.drop){

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else {
                if (fastTransfer){
                    collection.queueCommand(collection.transferSlam);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDropSlam);
                }else {
                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDrop);
                }

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);
            }

        }else if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.getSlidePositionCM() < 50 && delivery.slideTarget != delivery.highBasket && collection.getCurrentCommand() == collection.defaultCommand){

            delivery.slideSetPoint(delivery.highBasket);
            delivery.slides = Delivery.slideState.moving;

        }else if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.getSlidePositionCM() > 50 && delivery.fourbarState == Delivery.fourBarState.transfer){

            delivery.queueCommand(delivery.deposit);

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.slideMotor.getCurrentPosition() > 700){

            delivery.queueCommand(delivery.deposit);

        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("Four bar state ", collection.getFourBarState());
        telemetry.addData("rail position ", collection.getRailPosition());
        telemetry.addData("horizontal slides ", collection.getSlidePositionCM());
        telemetry.addData("vertical slides ", delivery.getSlidePositionCM());
        telemetry.addData("fastTransfer ", fastTransfer);
        telemetry.addData("delivery slides", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor collection", collection.clawSensor.isPressed());
        telemetry.addData("claw sensor delivery", delivery.clawSensor.isPressed());
        telemetry.addData("rail target", collection.getRailPosition());
        telemetry.addData("Resetting slides", collection.resettingSlides);
        telemetry.addData("Target point", collection.sampleMap.size());
        for (detectionData detection: collection.sampleMap){
            telemetry.addData("Target point", detection.getTargetPoint());
        }
        telemetry.update();
    }
}