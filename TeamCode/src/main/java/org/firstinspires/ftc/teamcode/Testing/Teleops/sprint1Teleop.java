package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection2;
import dev.weaponboy.command_library.Subsystems.Delivery;

@TeleOp
public class sprint1Teleop extends OpModeEX {

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();
    double turnPower = 0;
    double horizontal = 0;
    double lastHeading;

    double rotateTarget = 135;

    Servo hang1;
    Servo hang2;

    @Override
    public void initEX() {
        hang1 = hardwareMap.get(Servo.class,"hang1");
        hang2 = hardwareMap.get(Servo.class,"hang2");
        hang1.setDirection(Servo.Direction.REVERSE);

        hang1.setPosition(0.5);
        hang2.setPosition(0.5);
    }

    @Override
    public void loopEX() {

        horizontal = -gamepad2.right_stick_x*0.5;
        double lastHor = -lastGamepad2.right_stick_x*0.5;

        if (gamepad2.left_stick_x == 0 && Math.abs(horizontal) > 0){
            if (Math.abs(horizontal) > 0 && lastHor == 0){
                lastHeading = odometry.Heading();
            }
            turnPower = -driveBase.headindingLockMotorPower(lastHeading - odometry.Heading());
        }else {
            turnPower = -gamepad2.left_stick_x*0.5;
        }

        // drive base code
        driveBase.queueCommand(driveBase.drivePowers(-gamepad2.right_stick_y*0.5, turnPower, -gamepad2.right_stick_x*0.5));

//        //collection code
//        if (currentGamepad1.start && !lastGamepad1.start && delivery.getGripperState() == Delivery.gripper.drop){
//            delivery.setGripperState(Delivery.gripper.grab);
//        }else if (currentGamepad1.start && !lastGamepad1.start && delivery.getGripperState() == Delivery.gripper.grab){
//            delivery.setGripperState(Delivery.gripper.drop);
//        }

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper){
//            collection.queueCommand(collection.collect(20));
            collection.queueCommand(collection.collect);
            rotateTarget = 135;
        }

        if (currentGamepad1.back && !lastGamepad1.back){
            collection.overrideCurrent(true, collection.stow);
            delivery.overrideCurrent(true, delivery.stow);
        }

        if (currentGamepad1.left_stick_y < 0){
            collection.setSlideTarget(collection.getSlideTarget()+0.2);
        }else if (currentGamepad1.left_stick_y > 0){
            collection.setSlideTarget(collection.getSlideTarget()-0.2);
        }

        if (currentGamepad1.x && !lastGamepad1.x && delivery.slideMotor.getCurrentPosition() < 200){
            delivery.queueCommand(delivery.slideSetPonts(delivery.highChamber));
        }else if (currentGamepad1.x && !lastGamepad1.x && delivery.slideMotor.getCurrentPosition() > 200){
            delivery.queueCommand(delivery.Clip);
        }

        if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0) && collection.getFourBarState() == Collection2.fourBar.collect){
            collection.queueCommand(collection.transfer);
        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper &&delivery.fourbarState == Delivery.fourBarState.behindNest){
            delivery.queueCommand(delivery.transfer);
        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.fourbarState == Delivery.fourBarState.grabNest && delivery.getGripperState() == Delivery.gripper.grab && delivery.slideMotor.getCurrentPosition() < 1000){
            delivery.queueCommand(delivery.slideSetPonts(delivery.highBasket));
        }

        if (currentGamepad1.left_trigger > 0 && !(lastGamepad1.left_trigger > 0)){
            delivery.queueCommand(delivery.deposit);
        }

        if (gamepad1.right_stick_x > 0){
            rotateTarget += 1;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if (gamepad1.right_stick_x < 0){
            rotateTarget -= 1;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if (gamepad2.dpad_down){
            delivery.slideMotor.setPower(-0.4);
        }

//        if(gamepad1.dpad_left){
//            collection.setNestState(Collection2.Nest.specimen);
//        }else if(gamepad1.dpad_right){
//            collection.setNestState(Collection2.Nest.specimen);
//        }
//
//        if(gamepad1.dpad_left){
//            collection.linerRailServo.setPosition(1);
//        }else if(gamepad1.dpad_right){
//            collection.linerRailServo.setPosition(0);
//        }else{
//            collection.linerRailServo.setPosition(0.5);
//        }

        if (currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
            collection.setRailTargetPosition(8);
        }
//
        if (currentGamepad1.y && !lastGamepad1.y) {
            delivery.genProfile(0);
            delivery.queueCommand(delivery.followMotionPro);
            delivery.slidesState = Delivery.slideState.moving;
        }

        if (gamepad2.right_bumper){
            hang1.setPosition(0);
            hang2.setPosition(0);
        }else if (gamepad2.left_bumper){
            hang1.setPosition(1);
            hang2.setPosition(1);
        }else{
            hang1.setPosition(0.5);
            hang2.setPosition(0.5);
        }

//        if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0)) {
//            delivery.genProfile(0);
//            delivery.queueCommand(delivery.followMotionPro);
//            delivery.slidesState = Delivery.slideState.moving;
//        }

        telemetry.addData("rotateTarget", collection.griperRotate.getPositionDegrees());
        telemetry.addData("error", lastHeading - odometry.Heading());
        telemetry.addData("loop time ", loopTime);
        telemetry.addData("rail position ", collection.getRailPosition());
        telemetry.addData("horizontal slides ", collection.horizontalMotor.getCurrentPosition()/(440/35));
        telemetry.addData("vertical slides ", delivery.slideMotor.getCurrentPosition()/(2250/65));
        telemetry.addData("rail position ", collection.railEncoder.getCurrentPosition());

//        telemetry.addData("vertical slides ", delivery.slideMotor.getCurrentPosition());
        telemetry.update();
    }
}