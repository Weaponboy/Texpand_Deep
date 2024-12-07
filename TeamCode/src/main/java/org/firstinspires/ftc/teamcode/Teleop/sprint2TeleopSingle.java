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
public class sprint2TeleopSingle extends OpModeEX {

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();

    double rotateTarget = 90;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();

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
        driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.65, (gamepad1.left_trigger - gamepad1.right_trigger)*0.7, -gamepad1.right_stick_x*0.7));

        /**
         * Overwrites
         * */
        if (gamepad1.back){
            collection.sampleSorterContour.setScanning(false);
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
        }

        /**
         * Collection code
         * */
        if (gamepad1.x && (collection.getFourBarState() == Collection.fourBar.transferUp || collection.getFourBarState() == Collection.fourBar.preCollect)){
            collection.setSlideTarget(30);
        }

        if (currentGamepad1.a && !lastGamepad1.a){
            if (!delivery.clawSensor.isPressed()){
                delivery.setGripperState(Delivery.gripper.drop);
            }
            collection.queueCommand(collection.collect);
        }

        if (currentGamepad1.left_stick_y < -0.5){
            collection.setSlideTarget(collection.getSlideTarget()+0.5);
        }else if (currentGamepad1.left_stick_y > 0.5){
            collection.setSlideTarget(collection.getSlideTarget()-0.5);
        }

        if (currentGamepad1.left_stick_x < 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.setRailTargetPosition(collection.getRailPosition()-0.2);
        }else if (currentGamepad1.left_stick_x > 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.setRailTargetPosition(collection.getRailPosition()+0.2);
        }

        if (gamepad1.dpad_left){
            rotateTarget = 90;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if (gamepad1.dpad_right){
            rotateTarget = 180;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if(currentGamepad1.b && !lastGamepad1.b){
            delivery.queueCommand(delivery.cameraScan);
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

            delivery.mainPivot.setPosition(delivery.findCameraScanPosition(true));

            collection.sampleSorterContour.setScanning(true);
            collection.portal.resumeStreaming();

            busyDetecting = true;
            detectionTimer.reset();

        }

        if (busyDetecting && detectionTimer.milliseconds() > 1000 && !collection.sampleSorterContour.detections.isEmpty()){
            busyDetecting = false;
            collection.sampleSorterContour.setScanning(false);
            collection.portal.stopStreaming();
            collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

            collection.queueCommand(collection.autoCollectGlobal);
            collection.setChamberCollect(false);

            delivery.overrideCurrent(true, delivery.stow);
            delivery.runReset();
        }

//        if (!collection.sampleSorterContour.isScanning()){
//            collection.portal.stopStreaming();
//        }else {
//            collection.portal.resumeStreaming();
//        }

        /**
         * Delivery code
         * */
        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed()){
            delivery.queueCommand(delivery.preClip);
        }else if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() > 100){
            delivery.queueCommand(delivery.Clip);
        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer){

//            delivery.queueCommand(delivery.transfer);
//
//            transferring = true;
//
//            transferringWait.reset();

            delivery.queueCommand(delivery.transfer);

            delivery.queueCommand(collection.transferDrop);

            delivery.queueCommand(delivery.transfer);

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.slideMotor.getCurrentPosition() < 700){

            delivery.slideSetPoint(68);
            delivery.slides = Delivery.slideState.moving;

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.slideMotor.getCurrentPosition() > 700){
            delivery.queueCommand(delivery.deposit);
        }

        if (collection.clawSensor.isPressed() && collection.getFourBarState() == Collection.fourBar.transferUp && !transferring && collection.slidesReset.isPressed()){

            delivery.queueCommand(delivery.transfer);

            delivery.queueCommand(collection.transferDrop);

            delivery.queueCommand(delivery.transfer);

            transferring = true;

        } else if (!collection.clawSensor.isPressed() && transferring && delivery.getCurrentCommand() != delivery.transfer) {
            transferring = false;
        }

//        if (transferring && delivery.fourBarTargetState == Delivery.fourBarState.grabNest && delivery.getGripperState() == Delivery.gripper.grab && transferringWait.milliseconds() > 500){
//            collection.setClawsState(Collection.clawState.drop);
//            collection.Stowed.execute();
//            transferring = false;
//        }

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