//package org.firstinspires.ftc.teamcode.Testing.Teleops;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
//import dev.weaponboy.command_library.Subsystems.Collection;
//import dev.weaponboy.command_library.Subsystems.Delivery;
//
//@TeleOp
//public class FirstSprintOneDriver extends OpModeEX {
//
//    boolean colWaitForStow = false;
//    boolean colBusyCollecting = false;
//    boolean colWaitForTransfer = false;
//    boolean colWaitForDrop = false;
//    boolean depWaitforGrip = false;
//    boolean depWaitforTakeOutOfTansfer = false;
//    boolean depWaitForDrop = false;
//    boolean waitForColection = false;
//    boolean waitForRetern = false;
//
//    ElapsedTime collectionTimer = new ElapsedTime();
//    ElapsedTime stowTimer = new ElapsedTime();
//    ElapsedTime transferTime = new ElapsedTime();
//    ElapsedTime gripTime = new ElapsedTime();
//    ElapsedTime depGripTime = new ElapsedTime();
//    ElapsedTime depTakeFromTransferTime = new ElapsedTime();
//    ElapsedTime depDropTime = new ElapsedTime();
//    ElapsedTime collectionMoveTimer = new ElapsedTime();
//    ElapsedTime reternTimer = new ElapsedTime();
//
//    double targetPos = 128;
//
//    @Override
//    public void initEX() {
//
//    }
//
//    @Override
//    public void loopEX() {
//
//        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && collection.getCollectionState()== Collection.fourBar.stowed) {
//            collection.queueCommand(collection.preCollect);
//        }
//
//        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && collection.getCollectionState()== Collection.fourBar.preCollect) {
//            collection.queueCommand(collection.collect);
//            collection.setNestState(Collection.Nest.sample);
//        }
//
//        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && collection.getCollectionState()== Collection.fourBar.collect) {
//            collection.setGripperState(Collection.gripper.grab);
//
//            colBusyCollecting = true;
//            collectionTimer.reset();
//        }
//
//        if (colBusyCollecting && collectionTimer.milliseconds() > 500){
//            collection.queueCommand(collection.stow);
//            colBusyCollecting =false;
//            colWaitForStow =true;
//            stowTimer.reset();
//        }
//
//        if (colWaitForStow && stowTimer.milliseconds() > 450){
//            targetPos = 128;
//            collection.queueCommand(collection.transfer);
//            colWaitForStow =false;
//            colWaitForTransfer =true;
//
//            transferTime.reset();
//        }
//
//        if (colWaitForTransfer && transferTime.milliseconds() > 250){
//            collection.setGripperState(Collection.gripper.drop);
//            colWaitForTransfer =false;
//            colWaitForDrop =true;
//            gripTime.reset();
//        }
//
//        if (colWaitForDrop && gripTime.milliseconds() > 300){
//            collection.queueCommand(collection.stow);
//            colWaitForDrop =false;
//            gripTime.reset();
//        }
//
//        if (currentGamepad1.left_bumper && lastGamepad1.left_bumper && delivery.depositstate == Delivery.deposit.preTransFer){
//            collection.queueCommand(collection.init);
//            waitForColection =true;
//            collectionTimer.reset();
//        }
//        if (waitForColection && collectionTimer.milliseconds()>200) {
//            delivery.queueCommand(delivery.transfer);
//            waitForColection =false;
//            depWaitforGrip =true;
//            depGripTime.reset();
//        }
//
//        if (depWaitforGrip && depGripTime.milliseconds()>400){
//            delivery.queueCommand(delivery.postTransfer);
//            depWaitforGrip =false;
//            depWaitforTakeOutOfTansfer = true;
//            depTakeFromTransferTime.reset();
//        }
//
//        if (currentGamepad1.left_bumper&&lastGamepad1.left_bumper&&delivery.depositstate == Delivery.deposit.postTransfer&&collection.getNestState()== Collection.Nest.sample){
//            delivery.queueCommand(delivery.dropOff);
//            depWaitforTakeOutOfTansfer =false;
//        }
//        if (currentGamepad1.left_bumper&&lastGamepad1.left_bumper&&delivery.depositstate == Delivery.deposit.postTransfer&&collection.getNestState()== Collection.Nest.specimen){
//            delivery.queueCommand(delivery.cliping);
//            depWaitforTakeOutOfTansfer =false;
//        }
//
//
//        if (currentGamepad1.a&&lastGamepad1.a&&delivery.depositstate == Delivery.deposit.basket){
//            delivery.queueCommand(delivery.drop);
//            depWaitForDrop =true;
//            depDropTime.reset();
//        }
//
//        if (depWaitForDrop&&depDropTime.milliseconds()>150){
//            delivery.secondPivot.setPosition(258);
//            delivery.griperSev.setPosition(90);
//            depWaitForDrop =false;
//            waitForRetern=true;
//            reternTimer.reset();
//
//        }
//
//        if (waitForRetern&&reternTimer.milliseconds()>130){
//            waitForRetern =false;
//            delivery.mainPivot.setPosition(80);
//            delivery.depositstate = Delivery.deposit.preTransFer;
//
//        }
//
//        if (gamepad1.dpad_up){
//            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)+15);
//        }
//
//        if (gamepad1.b){
//            collection.setNestState(Collection.Nest.specimen);
//            collection.queueCommand(collection.init);
//        }
//
//        if (gamepad1.dpad_down){
//            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)-15);
//        }
//
////        if (currentGamepad1.x&&lastGamepad1.x){
////            collection.queueCommand(collection.PID);
////        }
//
//
//        if (gamepad1.back && collection.getCollectionState() == Collection.fourBar.collect){
//            collection.queueCommand(collection.preCollect);
//        }else if(gamepad1.back && delivery.depositstate ==Delivery.deposit.postTransfer){
//            delivery.queueCommand(delivery.behindTransfer);
//        }
//
//        if (gamepad1.start){
//            collection.setGripperState(Collection.gripper.grab);
//        }
//
////        if (gamepad1.back){
////            colection.queueCommand(colection.drop);
////        }
////
////        if (gamepad1.back){
////            delivery.queueCommand(delivery.drop);
////        }
//
//        driveBase.queueCommand(driveBase.drivePowers(-gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
//
//        if (collection.horizontalMotor.getCurrentPosition()< collection.maxSlideExtension) {
//            collection.setExtendoPower(-gamepad1.left_stick_y/2);
//        }
////        if (delivery.slidesState== Delivery.slideState.holdPosition) {
////            if (gamepad1.right_trigger > 0 && Math.abs(delivery.slideMotor.getCurrentPosition()) < delivery.maxSlideHeight) {
////                delivery.slideMotor.setPower(0.05 + gamepad1.right_trigger);
////            } else if (gamepad1.left_trigger > 0 && delivery.slideMotor.getCurrentPosition() > 90) {
////                delivery.slideMotor.setPower(0.05 - gamepad1.left_trigger);
////            } else {
////                delivery.queueCommand(delivery.holdPosition);
////            }
////        }
//        if (currentGamepad1.y && !lastGamepad1.y){
//            delivery.genProfile(delivery.highBasket);
//            delivery.queueCommand(delivery.followMotionPro);
//            delivery.slidesState= Delivery.slideState.moving;
////            double motorPower = 0;
////            while (delivery.slideMotor.getCurrentPosition() < 400){
////                motorPower += 0.05;
////                delivery.slideMotor.setPower(motorPower);
////            }
////
////            delivery.slideMotor.setPower(0.36);A
//
//        }
//        if (currentGamepad1.left_stick_button && !lastGamepad1.left_stick_button){
//            delivery.genProfile(0);
//            delivery.queueCommand(delivery.followMotionPro);
//            delivery.slidesState= Delivery.slideState.moving;
////            double motorPower = 0;
////            while (delivery.slideMotor.getCurrentPosition() < 400){
////                motorPower += 0.05;
////                delivery.slideMotor.setPower(motorPower);
////            }
////
////            delivery.slideMotor.setPower(0.36);A
//
//        }
////        if(gamepad1.dpad_left){
////            collection.linerRailServo.setPosition(0);
////        }else if(gamepad1.dpad_right){
////            collection.linerRailServo.setPosition(1);
////        }else{
////            collection.linerRailServo.setPosition(0.5);
////        }
//
//        if(gamepad1.dpad_left){
//            collection.setRailTargetPosition(10);
//        }
//        collection.updateRailPosition();
//
//        telemetry.addData("horPos", collection.horizontalMotor.getCurrentPosition());
//        telemetry.addData("looptime ", loopTime);
////        telemetry.addData("vertPos",delivery.slideMotor.getCurrentPosition()*delivery.p);
//        telemetry.addData("axon_pos",collection.linearPosition.getPosition());
//        telemetry.addData("linear Rail Position",collection.getRailPosition());
//        telemetry.addData("vertPow",delivery.slideMotor.getPower());
//
//        telemetry.update();
//    }
//}