package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.acl.Group;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;
import dev.weaponboy.vision.detectionData;

@TeleOp(name = "AatsTeleop", group = "AAAAAAt the top")
public class natsTeleop extends OpModeEX {

    boolean cameraScan = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    boolean fastTransfer = true;
    boolean queueCollection = false;

    boolean ranTransfer = false;

    boolean autoPreClip = false;
    boolean ranPreClip = false;

    @Override
    public void initEX() {
        FtcDashboard.getInstance().startCameraStream(collection.sampleDetector, 30);

        odometry.startPosition(82.5,100,0);

        collection.sampleDetector.setTargetColor(findAngleUsingContour.TargetColor.yellow);
        collection.sampleDetector.closestFirst = true;
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
            collection.sampleDetector.setScanning(false);
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
            collection.targetPosition = new Vector2D(20, 20);

            cameraScan = false;
            ranTransfer = false;
        }

        if (gamepad1.dpad_left){
            collection.angle = 70;

            collection.queueCommand(collection.extendoTargetPoint(new Vector2D(130, 100)));
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

            ranTransfer = false;
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

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));
            collection.targetPosition = new Vector2D(20, 20);

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else {
                if (fastTransfer){
                    collection.queueCommand(collection.transferSlam);

                    collection.queueCommand(collection.transferDropSlam);
                }else {
                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(collection.transferDrop);
                }

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                ranTransfer = true;
            }
        }

        if (currentGamepad2.left_stick_y < -0.4){
            collection.armEndPointIncrement(0, 0.5, false);
        }

        if (currentGamepad2.left_stick_y > 0.4){
            collection.armEndPointIncrement(0, -0.5, false);
        }

        if (currentGamepad2.left_stick_x < -0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(-Math.abs(currentGamepad2.left_stick_x*0.5), 0, false);
        }else if (currentGamepad2.left_stick_x > 0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(Math.abs(currentGamepad2.left_stick_x*0.5), 0, false);
        }

        if (currentGamepad2.left_trigger > 0.1 && lastGamepad2.left_trigger < 0.1 && collection.manualAngle < 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.manualAngle = 90;
            collection.armEndPointIncrement(0, 0, false);
        }else if (currentGamepad2.left_trigger > 0.1 && lastGamepad2.left_trigger < 0.1 && collection.manualAngle > 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.manualAngle = 0;
            collection.armEndPointIncrement(0, 0, false);
        }

        if (gamepad2.dpad_left){
            hang.hang1.setPosition(1);
            hang.hang2.setPosition(1);
        }else if (gamepad2.dpad_right){
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

            cameraScan = false;
            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;

        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;

            if (limelight.getTargetPoint() != null){

                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));
                collection.setChamberCollect(false);

                delivery.overrideCurrent(true, delivery.stow);
                delivery.runReset();

                queueCollection = true;
                busyDetecting = false;

                counter = 40;
            }

        } else if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter > 20) {

            collection.sampleDetector.setScanning(false);
//            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();

            busyDetecting = false;
        }

        if (collection.sampleDetector.isScanning() && !collection.sampleDetector.detections.isEmpty() && delivery.getSlidePositionCM() > 10 && odometry.getXVelocity() < 10){
            gamepad2.rumble(5);
        }

        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));

            collection.targetPosition = new Vector2D(20, 20);

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else {
                if (fastTransfer){
                    collection.queueCommand(collection.transferSlam);

//                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDropSlam);
                }else {
                    collection.queueCommand(collection.transfer);

//                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDrop);
                }

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);
            }

            queueCollection = false;
            ranTransfer = true;
        }

        if (currentGamepad1.start && lastGamepad1.start && autoPreClip){
            autoPreClip = false;
            ranPreClip = false;
            gamepad1.rumble(300);
        }else if (currentGamepad1.start && lastGamepad1.start && !autoPreClip){
            autoPreClip = true;
            ranPreClip = false;
            gamepad1.rumble(300);
        }

        /**
         * Delivery code
         * */
        if (ranTransfer && !ranPreClip && autoPreClip && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed() && collection.getCurrentCommand() == collection.defaultCommand){
            delivery.queueCommand(delivery.preClipFront);
            delivery.griperRotateSev.setPosition(90);

            ranPreClip = true;
            ranTransfer = false;
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

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));

            collection.targetPosition = new Vector2D(20, 20);

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else if(collection.getFourBarState() == Collection.fourBar.stowedChamber){
                collection.setClawsState(Collection.clawState.drop);
            }else {
                if (fastTransfer){
                    collection.queueCommand(collection.transferSlam);

//                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDropSlam);
                }else {
                    collection.queueCommand(collection.transfer);

//                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDrop);
                }

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                ranTransfer = true;
            }

        }

        if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.drop){

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));

            collection.targetPosition = new Vector2D(20, 20);

            if (collection.getChamberCollect()){
                collection.queueCommand(collection.chamberCollect);
            }else {
                if (fastTransfer){
                    collection.queueCommand(collection.transferSlam);

                    collection.queueCommand(collection.transferDropSlam);
                }else {
                    collection.queueCommand(collection.transfer);

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
        telemetry.addData("pre clip thing ", autoPreClip);
        telemetry.addData("ran pre clip thing ", ranPreClip);
        telemetry.addData("ranTransfer ", ranTransfer);
        telemetry.addData("Four bar state ", collection.getFourBarState());
        telemetry.addData("horizontal slides ", collection.getSlidePositionCM());
        telemetry.addData("vertical slides 1 ", delivery.getSlidePositionCM());
        telemetry.addData("vertical slides 2 ", delivery.slideMotor2.getCurrentPosition());
        telemetry.addData("fastTransfer ", fastTransfer);
        telemetry.addData("delivery slides", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor collection", collection.breakBeam.isPressed());
        telemetry.addData("claw sensor delivery", delivery.clawSensor.isPressed());
        telemetry.addData("Resetting slides", collection.manualAngle);
        telemetry.addData("Target point", collection.getSlideTarget());
        if (limelight.getTargetPoint() != null){
            telemetry.addData("limelight results X", limelight.getTargetPoint().getTargetPoint().getX());
            telemetry.addData("limelight results Y", limelight.getTargetPoint().getTargetPoint().getY());
            telemetry.addData("limelight results angle", limelight.getTargetPoint().getAngle());
        }
        telemetry.update();
    }
}