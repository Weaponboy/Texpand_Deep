package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;

@TeleOp
public class firstSprintTeliop extends OpModeEX {
    boolean colWaitForStow = false;
    boolean colBusyCollecting = false;
    boolean colWaitForTransfer = false;
    boolean colWaitForDrop = false;
    boolean depWaitforGrip = false;
    boolean depWaitforTakeOutOfTansfer = false;
    boolean depWaitForDrop;
    boolean waitForCollection =false;

    ElapsedTime collectionTimer = new ElapsedTime();
    ElapsedTime stowTimer = new ElapsedTime();
    ElapsedTime transferTime = new ElapsedTime();
    ElapsedTime gripTime = new ElapsedTime();
    ElapsedTime depGripTime = new ElapsedTime();
    ElapsedTime depTakeFromTransferTime = new ElapsedTime();
    ElapsedTime depDropTime = new ElapsedTime();
    ElapsedTime collectionMoveTimer = new ElapsedTime();
    double targetPos = 128;

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {


        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && collection.collectionState== Collection.fourBar.stowed) {
            collection.queueCommand(collection.preCollect);
        }

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && collection.collectionState== Collection.fourBar.preCollect) {
            collection.queueCommand(collection.Collect);
        }

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && collection.collectionState== Collection.fourBar.collect) {
            collection.queueCommand(collection.grip);

            colBusyCollecting = true;
            collectionTimer.reset();
        }

        if (colBusyCollecting && collectionTimer.milliseconds() > 400){
            collection.queueCommand(collection.stow);
            colBusyCollecting =false;
            colWaitForStow =true;
            stowTimer.reset();
        }
        if (colWaitForStow && stowTimer.milliseconds() > 600){
            targetPos = 128;
            collection.queueCommand(collection.transfer);
            colWaitForStow =false;
            colWaitForTransfer =true;

            transferTime.reset();
        }

        if (colWaitForTransfer && transferTime.milliseconds() > 300){
            collection.queueCommand(collection.drop);
            colWaitForTransfer =false;
            colWaitForDrop =true;
            gripTime.reset();
        }

        if (colWaitForDrop && gripTime.milliseconds() > 500){
            collection.queueCommand(collection.stow);
            colWaitForDrop =false;
            gripTime.reset();
        }
        if (currentGamepad1.y&&lastGamepad1.y&&delivery.depositstate == Delivery.deposit.preTransFer){
            collection.queueCommand(collection.init);
            waitForCollection =true;
            collectionTimer.reset();
        }
        if (waitForCollection && collectionTimer.milliseconds()>250) {
            delivery.queueCommand(delivery.transfer);
            waitForCollection =false;
            depWaitforGrip =true;
            depGripTime.reset();
        }

        if (depWaitforGrip && depGripTime.milliseconds()>1000){
            delivery.queueCommand(delivery.postTransfer);
            depWaitforGrip =false;
            depWaitforTakeOutOfTansfer = true;
            depTakeFromTransferTime.reset();
        }
//        if (currentGamepad1.y&&lastGamepad1.y&&delivery.depositstate == Delivery.deposit.postTransfer){
//            delivery.queueCommand(delivery.dropOff);
//            depWaitforTakeOutOfTansfer =false;
//        }
        if (currentGamepad1.left_bumper&&lastGamepad1.left_bumper&&delivery.depositstate == Delivery.deposit.basket){
            delivery.queueCommand(delivery.drop);
            depWaitForDrop =true;
            depDropTime.reset();

        }
        if (depWaitForDrop&&depDropTime.milliseconds()>200){
            depWaitForDrop =false;
            delivery.queueCommand(delivery.behindTransfer);
        }

        if (gamepad1.dpad_up){
            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)+15);
        }
        if (gamepad1.dpad_down){
            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)-15);
        }
        if (gamepad1.back&&collection.collectionState == Collection.fourBar.collect ){
            collection.queueCommand(collection.preCollect);
        }
        if (gamepad1.start){
            collection.queueCommand(collection.grip);
        }

//        if (gamepad1.back){
//            collection.queueCommand(collection.drop);
//        }
//
//        if (gamepad1.back){
//            delivery.queueCommand(delivery.drop);
//        }

        driveBase.drivePowers(-gamepad2.right_stick_y, gamepad2.left_stick_x,gamepad2.right_stick_x);

        if (collection.horizontalMotor.getCurrentPosition()< collection.maxSlideExtension) {
            collection.horizontalMotor.setPower(-gamepad1.right_stick_y/2);
        }

        if (gamepad1.left_stick_y > 0 && Math.abs(delivery.slideMotor.getCurrentPosition()) < delivery.maxSlideHeight){
            delivery.slideMotor.setPower(0.05+gamepad1.left_stick_y);
        }else if(gamepad1.left_stick_y < 0 && Math.abs(delivery.slideMotor.getCurrentPosition()) > 10){
            delivery.slideMotor.setPower(0.05+gamepad1.left_stick_y);
        }else{
            delivery.queueCommand(delivery.holdPosition);
        }

        if (currentGamepad1.y && !lastGamepad1.y){
            delivery.genProfile(delivery.lowBasket);
//            delivery.queueCommand(delivery.followMotionPro);
//            double motorPower = 0;
//            while (delivery.slideMotor.getCurrentPosition() < 400){
//                motorPower += 0.05;
//                delivery.slideMotor.setPower(motorPower);
//            }
//
//            delivery.slideMotor.setPower(0.36);A

        }
        if(gamepad1.dpad_left){
            collection.linerRailServo.setPosition(0);
        }else if(gamepad1.dpad_right){
            collection.linerRailServo.setPosition(1);
        }else{
            collection.linerRailServo.setPosition(0.5);
        }

//        collection.setLinearRailPos(targetPos);
        System.out.println("Testing");

        telemetry.addData("horPos", collection.horizontalMotor.getCurrentPosition());
//        telemetry.addData("vertPos power",delivery.slideMotor.getPower());
        telemetry.addData("looptime ", loopTime);
        telemetry.addData("vertPos",delivery.slideMotor.getCurrentPosition());
        telemetry.addData("axon_pos",collection.linearPosition.getPosition());
        telemetry.update();
        }
}
