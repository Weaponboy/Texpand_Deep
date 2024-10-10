package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection2;
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
        driveBase.queueCommand(driveBase.drivePowers(-gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

//        //collection code
//        if (currentGamepad1.start && !lastGamepad1.start && delivery.getGripperState() == Delivery.gripper.drop){
//            delivery.setGripperState(Delivery.gripper.grab);
//        }else if (currentGamepad1.start && !lastGamepad1.start && delivery.getGripperState() == Delivery.gripper.grab){
//            delivery.setGripperState(Delivery.gripper.drop);
//        }

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper){
            collection.queueCommand(collection.collect(20));
        }

        if (currentGamepad1.back && !lastGamepad1.back){
            collection.overrideCurrent(true, collection.stow);
            delivery.overrideCurrent(true, delivery.stow);
        }

        if (currentGamepad1.b && !lastGamepad1.b){
            delivery.genProfile(0);
            delivery.queueCommand(delivery.followMotionPro);
            delivery.slidesState = Delivery.slideState.moving;
        }

        if (currentGamepad1.x && !lastGamepad1.x){
            delivery.queueCommand(delivery.Clip);
        }

        if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0) && collection.getFourBarState() == Collection2.fourBar.collect){
            collection.queueCommand(collection.transfer);
        }

//        if (gamepad1.start){
//            collection.disableServos();
//        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper){
            delivery.queueCommand(delivery.transfer);
        }

        if (currentGamepad1.left_trigger > 0 && !(lastGamepad1.left_trigger > 0)){
            delivery.queueCommand(delivery.deposit);
        }

//        if (gamepad1.dpad_up){
//            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)+15);
//        }else if (gamepad1.dpad_down){
//            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)-15);
//        }

//        if(gamepad1.dpad_left){
//            collection.linerRailServo.setPosition(0);
//        }else if(gamepad1.dpad_right){
//            collection.linerRailServo.setPosition(1);
//        }else{
//            collection.linerRailServo.setPosition(0.5);
//        }

        if (currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
            collection.setRailTargetPosition(10);
        }
//
//        if (currentGamepad1.y && !lastGamepad1.y) {
//            delivery.genProfile(delivery.highBasket);
//            delivery.queueCommand(delivery.followMotionPro);
//            delivery.slidesState = Delivery.slideState.moving;
//        }

//        if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0)) {
//            delivery.genProfile(0);
//            delivery.queueCommand(delivery.followMotionPro);
//            delivery.slidesState = Delivery.slideState.moving;
//        }

        telemetry.addData("loop time ", loopTime);
//        telemetry.addData("rail position ", collection.getRailPosition());
        telemetry.addData("horizontal slides ", collection.horizontalMotor.getCurrentPosition()/(440/35));
        telemetry.addData("horizontal slides ", collection.fourBarMainPivot.getPosition());
        telemetry.addData("horizontal slides ", collection.fourBarSecondPivot.getPosition());
        telemetry.addData("horizontal slides ", collection.griperRotate.getPosition());
        telemetry.addData("rail position ", collection.linearPosition.getPosition());

//        telemetry.addData("vertical slides ", delivery.slideMotor.getCurrentPosition());
        telemetry.update();
    }
}