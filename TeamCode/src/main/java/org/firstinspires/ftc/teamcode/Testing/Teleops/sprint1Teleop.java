package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Delivery;

@TeleOp
public class sprint1Teleop extends OpModeEX {

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();
    double waitTime = 0;

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {

        // drive base code
        driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x));

//        //collection code
//        if (currentGamepad1.start && !lastGamepad1.start && delivery.getGripperState() == Delivery.gripper.drop){
//            delivery.setGripperState(Delivery.gripper.grab);
//        }else if (currentGamepad1.start && !lastGamepad1.start && delivery.getGripperState() == Delivery.gripper.grab){
//            delivery.setGripperState(Delivery.gripper.drop);
//        }

        if (currentGamepad1.right_bumper && !lastGamepad1.left_bumper){
            collection.queueCommand(collection.collect);
        }

        if (currentGamepad1.back && !lastGamepad1.back){
            collection.queueCommand(collection.stow);
        }

        if (currentGamepad1.left_trigger > 0 && !(lastGamepad1.left_trigger > 0)){
            collection.queueCommand(collection.transfer);
        }

        if (gamepad1.start){
            collection.disableServos();
        }

//        if (currentGamepad1.right_bumper && !currentGamepad1.left_bumper && collection.getCollectionState() == Collection.fourBar.stowed){
//            collection.queueCommand(collection.preCollect);
//        }else if (currentGamepad1.right_bumper && !currentGamepad1.left_bumper && collection.getCollectionState() == Collection.fourBar.preCollect){
//            collection.queueCommand(collection.collect);
//        }else if (currentGamepad1.right_bumper && !currentGamepad1.left_bumper && collection.getCollectionState() == Collection.fourBar.collect){
//            collection.setGripperState(Collection.gripper.grab);
//            collection.setNestState(Collection.Nest.sample);
//            transferring = true;
//            waitTime = 400;
//            transferringWait.reset();
//        }

//        if (collection.getCollectionState() == Collection.fourBar.stowed && transferring && transferringWait.milliseconds() > waitTime){
//            collection.queueCommand(collection.dropNest);
//            waitTime = 400;
//            transferringWait.reset();
//        }else if (collection.getCollectionState() == Collection.fourBar.dropNest && transferring && transferringWait.milliseconds() > waitTime){
//            collection.setGripperState(Collection.gripper.grab);
//            waitTime = 400;
//            transferringWait.reset();
//        }else if (collection.getCollectionState() == Collection.fourBar.stowed && transferring && transferringWait.milliseconds() > waitTime && collection.getGripperState() == Collection.gripper.drop){
//            collection.queueCommand(collection.stow);
//            transferring = false;
//        }

//        if (gamepad1.dpad_up){
//            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)+15);
//        }else if (gamepad1.dpad_down){
//            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)-15);
//        }

//        if (collection.horizontalMotor.getCurrentPosition() < collection.maxSlideExtension && collection.getSlidesState() == Collection.slideState.manuel) {
//            collection.setExtendoPower(-gamepad1.left_stick_y/2);
//        } else if (collection.getSlidesState() == Collection.slideState.manuel) {
//            collection.setExtendoPower(-gamepad1.left_stick_y/2);
//        }

        if(gamepad1.dpad_left){
            collection.linerRailServo.setPosition(0);
        }else if(gamepad1.dpad_right){
            collection.linerRailServo.setPosition(1);
        }else{
            collection.linerRailServo.setPosition(0.5);
        }

        if (gamepad1.dpad_up){
            collection.setRailTargetPosition(10);
        }

        if (currentGamepad1.y && !lastGamepad1.y) {
            delivery.genProfile(delivery.highBasket);
            delivery.queueCommand(delivery.followMotionPro);
            delivery.slidesState = Delivery.slideState.moving;
        }

        if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0)) {
            delivery.genProfile(0);
            delivery.queueCommand(delivery.followMotionPro);
            delivery.slidesState = Delivery.slideState.moving;
        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("rail position ", collection.getRailPosition());
        telemetry.addData("rail position ", collection.linearPosition.getPosition());
        telemetry.update();
    }
}