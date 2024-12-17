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

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();

    double rotateTarget = 90;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

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
        driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.65, (-gamepad1.left_stick_x)*0.55, -gamepad1.right_stick_x*0.7));

        /**
         * Overwrites
         * */
        if (currentGamepad2.back && !lastGamepad1.back){
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
            if (!delivery.clawSensor.isPressed()){
                delivery.setGripperState(Delivery.gripper.drop);
            }

            collection.queueCommand(collection.collect);
        }

        if (collection.clawSensor.isPressed() && collection.getCurrentCommand() == collection.transfer){
            collection.setChamberCollect(false);
        }

        if (currentGamepad2.right_trigger > 0 && !(lastGamepad2.right_trigger > 0) && collection.getFourBarState() == Collection.fourBar.collect){

            collection.queueCommand(collection.transfer);

//            collection.queueCommand(delivery.transfer);

            collection.queueCommand(collection.transferDrop);

            collection.queueCommand(delivery.closeGripper);

            collection.queueCommand(collection.openGripper);

        }

        if (currentGamepad2.right_stick_y < -0.5){
            collection.setSlideTarget(collection.getSlideTarget()+0.5);
        }else if (currentGamepad2.right_stick_y > 0.5){
            collection.setSlideTarget(collection.getSlideTarget()-0.5);
        }

        if (currentGamepad2.left_stick_x < 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.setRailTargetPosition(collection.getRailPosition()-0.2);
        }else if (currentGamepad2.left_stick_x > 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.setRailTargetPosition(collection.getRailPosition()+0.2);
        }

        if (currentGamepad2.dpad_left){
            rotateTarget = 90;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if (currentGamepad2.dpad_right){
            rotateTarget = 180;
            collection.griperRotate.setPosition(rotateTarget);
        }

//        if (gamepad2.dpad_left){
//            collection.setChamberCollect(false);
//        }
//
//        if (gamepad2.dpad_right){
//            collection.setChamberCollect(true);
//        }

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
        }

        if (currentGamepad2.y && !lastGamepad2.y){

            delivery.mainPivot.setPosition(delivery.findCameraScanPosition(true));

            collection.sampleSorterContour.setScanning(true);
            collection.portal.resumeStreaming();

            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;

        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;

            if (!collection.sampleSorterContour.detections.isEmpty() && counter > 10){
                busyDetecting = false;
                collection.sampleSorterContour.setScanning(false);
                collection.portal.stopStreaming();
                collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

                collection.queueCommand(collection.autoCollectGlobal);
                collection.setChamberCollect(false);

                delivery.overrideCurrent(true, delivery.stow);
                delivery.runReset();
            }

        } else if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter > 20) {

            collection.sampleSorterContour.setScanning(false);
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();

            busyDetecting = false;
        }

        /**
         * Delivery code
         * */
        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed()){
            delivery.queueCommand(delivery.preClipFront);
        }else if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() > 100){
            delivery.queueCommand(delivery.clipFront);
        }

        if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.drop){

            collection.queueCommand(delivery.transfer);

            collection.queueCommand(collection.transferDrop);

            collection.queueCommand(delivery.closeGripper);

            collection.queueCommand(collection.openGripper);

        }else if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.getSlidePositionCM() < 50 && delivery.slideTarget != delivery.highBasket){

            delivery.slideSetPoint(delivery.highBasket);
            delivery.slides = Delivery.slideState.moving;

        }else if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.getSlidePositionCM() > 50 && delivery.fourbarState == Delivery.fourBarState.transfer){

            delivery.queueCommand(delivery.deposit);

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.slideMotor.getCurrentPosition() > 700){

            delivery.queueCommand(delivery.deposit);

        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("rail position ", collection.getRailPosition());
        telemetry.addData("horizontal slides ", collection.horizontalMotor.getCurrentPosition());
        telemetry.addData("vertical slides ", delivery.getSlidePositionCM());
        telemetry.addData("collection current command ", collection.getCurrentCommand().toString());
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