package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.vision.detectionData;

@TeleOp
@Disabled
public class sprint2TeleopTesting extends OpModeEX {

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();

    double rotateTarget = 90;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    boolean raisingSlides = false;

    @Override
    public void initEX() {
        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        odometry.startPosition(1000,1000,0);

//        delivery.Deposit.execute();
        collection.sampleSorterContour.closestFirst = true;
    }

    @Override
    public void loopEX() {

//        horizontal = -gamepad2.right_stick_x*0.5;
//        double lastHor = -lastGamepad2.right_stick_x*0.5;
//
//        if (gamepad2.left_stick_x == 0 && Math.abs(horizontal) > 0){
//            if (Math.abs(horizontal) > 0 && lastHor == 0){
//                lastHeading = odometry.Heading();
//            }
//            turnPower = -driveBase.headindingLockMotorPower(lastHeading - odometry.Heading());
//        }else {
//            turnPower = -gamepad2.left_stick_x*0.5;
//            if (Math.abs(turnPower) > 0 && Math.abs(horizontal) > 0 ){
//                lastHeading = odometry.Heading();
//            }
//        }
        //-gamepad2.left_stick_x*0.5

        // drive base code
        driveBase.queueCommand(driveBase.drivePowers(gamepad2.right_stick_y*0.65, gamepad2.left_trigger - gamepad2.right_trigger, -gamepad2.right_stick_x*0.7));

        /**
         * Overwrites
         * */
        if (currentGamepad1.back && !lastGamepad1.back){
//            collection.overrideCurrent(true, collection.stow);
            delivery.overrideCurrent(true, delivery.stow);
            delivery.runReset();
        }

        /**
         * Collection code
         * */

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper){
            if (!delivery.clawSensor.isPressed()){
                delivery.setGripperState(Delivery.gripper.drop);
            }
            collection.queueCommand(collection.collect);
        }

        if (currentGamepad1.right_stick_y < -0.5){
            collection.setSlideTarget(collection.getSlideTarget()+0.5);
        }else if (currentGamepad1.right_stick_y > 0.5){
            collection.setSlideTarget(collection.getSlideTarget()-0.5);
        }

        if (currentGamepad1.right_stick_x < 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.setRailTargetPosition(collection.getRailPosition()-0.2);
        }else if (currentGamepad1.right_stick_x > 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.setRailTargetPosition(collection.getRailPosition()+0.2);
        }

        //first statement is the normal transfer in case the auto transfer fails
        //second statement is for drop and collect at the observation zone
        if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0) && collection.getFourBarState() == Collection.fourBar.collect){
            collection.queueCommand(collection.transfer);
            collection.setChamberCollect(false);
            rotateTarget = 90;
        }else if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0) && collection.getFourBarState() == Collection.fourBar.collectChamber){
            collection.setClawsState(Collection.clawState.drop);
            collection.setSlideTarget(collection.getSlideTarget()-12);
            collection.queueCommand(collection.autoCollectGlobal);
        }

        if (gamepad1.left_stick_x > 0 && rotateTarget > 0){
            rotateTarget -= 1;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if (gamepad1.left_stick_x < 0 && rotateTarget < 180){
            rotateTarget += 1;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if(currentGamepad1.b && !lastGamepad1.b){
            delivery.slideSetPonts(30);
            delivery.slides = Delivery.slideState.moving;

            raisingSlides = true;
        }else if (raisingSlides && delivery.slides == Delivery.slideState.holdPosition){
            delivery.secondPivot.setPosition(170);
            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
            collection.sampleSorterContour.setScanning(true);
        }

        if (currentGamepad1.y && !lastGamepad1.y) {
            if (collection.sampleSorterContour.isScanning()){
                raisingSlides = false;
                collection.sampleSorterContour.setScanning(false);
                collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(RobotPosition, delivery.getSlidePositionCM());
            }

            delivery.runReset();
            delivery.behindNest.execute();
            collection.queueCommand(collection.autoCollectGlobal);
            collection.setChamberCollect(true);
        }

        if (currentGamepad1.a && !lastGamepad1.a){
            if (collection.sampleSorterContour.isScanning()){
                raisingSlides = false;
                collection.sampleSorterContour.setScanning(false);
                collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(RobotPosition, delivery.getSlidePositionCM());
            }

            delivery.runReset();
            delivery.behindNest.execute();
            collection.queueCommand(collection.autoCollectGlobal);
            collection.setChamberCollect(false);
        }

        if (!collection.sampleSorterContour.isScanning()){
            collection.portal.stopStreaming();
        }else {
            collection.portal.resumeStreaming();
        }

        if (gamepad1.dpad_up){
            collection.queueCommand(collection.autoCollectGlobal);
        }

        if (currentGamepad1.dpad_left && !lastGamepad1.dpad_left && collection.getClawsState() == Collection.clawState.drop){
            collection.setClawsState(Collection.clawState.grab);
        }else  if (currentGamepad1.dpad_left && !lastGamepad1.dpad_left && collection.getClawsState() == Collection.clawState.grab){
            collection.setClawsState(Collection.clawState.drop);
        }

        /**
         * Delivery code
         * */

        if (currentGamepad1.left_trigger > 0 && !(lastGamepad1.left_trigger > 0)){
            delivery.queueCommand(delivery.deposit);
        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.fourbarState == Delivery.fourBarState.behindNest){

            delivery.queueCommand(delivery.transfer);

            transferring = true;

            transferringWait.reset();

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.fourbarState == Delivery.fourBarState.postTransfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.slideMotor.getCurrentPosition() < 1000){
            delivery.slideSetPonts(68);
            delivery.slides = Delivery.slideState.moving;
        }

        if (collection.clawSensor.isPressed() && !delivery.clawSensor.isPressed() && collection.getFourBarState() == Collection.fourBar.transferUp && !transferring && collection.slidesReset.isPressed()){
            delivery.queueCommand(delivery.transfer);

            transferring = true;

            transferringWait.reset();
        }

        if (transferring && delivery.fourBarTargetState == Delivery.fourBarState.grabNest && delivery.getGripperState() == Delivery.gripper.grab && transferringWait.milliseconds() > 500){
            collection.setClawsState(Collection.clawState.drop);
            transferring = false;
        }

//        if (!collection.nestSensor.isPressed() && delivery.slidesReset.isPressed() && collection.getCurrentCommand() != collection.transfer && collection.slidesReset.isPressed() && delivery.fourbarState == Delivery.fourBarState.behindNest){
//            delivery.queueCommand(delivery.transfer);
//        }

//        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.getClawsState() == Collection.clawState.drop){
//            collection.setClawsState(Collection.clawState.grab);
//        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.getClawsState() == Collection.clawState.grab){
//            collection.setClawsState(Collection.clawState.drop);
//        }1054/75

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