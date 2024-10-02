package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;

@TeleOp
public class sprint1Teleop extends OpModeEX {

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {

        // drive base code
        driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x));

        //collection code
        if (currentGamepad1.start && !lastGamepad1.start && collection.getGripperState() == Collection.gripper.drop){
            collection.queueCommand(collection.grab);
        }else if (currentGamepad1.start && !lastGamepad1.start && collection.getGripperState() == Collection.gripper.grab){
            collection.queueCommand(collection.drop);
        }

        if (currentGamepad1.right_bumper && !currentGamepad1.left_bumper && collection.getCollectionState() == Collection.fourBar.stowed){
            collection.queueCommand(collection.preCollect);
        }else if (currentGamepad1.right_bumper && !currentGamepad1.left_bumper && collection.getCollectionState() == Collection.fourBar.preCollect){
            collection.queueCommand(collection.collect);
        }else if (currentGamepad1.right_bumper && !currentGamepad1.left_bumper && collection.getCollectionState() == Collection.fourBar.collect){
            collection.queueCommand(collection.grab);
            collection.setNestState(Collection.Nest.sample);
        }

        if (gamepad1.dpad_up){
            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)+15);
        }else if (gamepad1.dpad_down){
            collection.griperRotate.setPosition((collection.griperRotate.getPosition()*270)-15);
        }

        if (collection.horizontalMotor.getCurrentPosition()< collection.maxSlideExtension) {
            collection.horizontalMotor.setPower(-gamepad1.left_stick_y/2);
        }

        if(gamepad1.dpad_left){
            collection.linerRailServo.setPosition(0);
        }else if(gamepad1.dpad_right){
            collection.linerRailServo.setPosition(1);
        }else{
            collection.linerRailServo.setPosition(0.5);
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
        telemetry.update();
    }
}