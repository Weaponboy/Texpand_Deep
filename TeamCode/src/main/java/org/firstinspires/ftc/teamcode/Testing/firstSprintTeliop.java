package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Colection;

@TeleOp
public class firstSprintTeliop extends OpModeEX {
    boolean waitForStow = false;
    boolean busyCollecting = false;
    boolean waitForTransfer = false;
    boolean waitForDrop = false;

    ElapsedTime collectionTimer = new ElapsedTime();
    ElapsedTime stowTimer = new ElapsedTime();
    ElapsedTime transferTime = new ElapsedTime();
    ElapsedTime gripTime = new ElapsedTime();

    double motor = 0;

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && colection.collectionState== Colection.fourBar.stowed) {
            colection.queueCommand(colection.preCollect);
        }

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && colection.collectionState== Colection.fourBar.preCollect) {
            colection.queueCommand(colection.Collect);
        }

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && colection.collectionState== Colection.fourBar.collect) {
            colection.queueCommand(colection.grip);

            busyCollecting = true;
            collectionTimer.reset();
        }

        if (busyCollecting && collectionTimer.milliseconds() > 400){
            colection.queueCommand(colection.stow);
            busyCollecting =false;
            waitForStow =true;
            stowTimer.reset();
        }
        if (waitForStow && stowTimer.milliseconds() > 600){
            colection.queueCommand(colection.transfer);
            waitForStow=false;
            waitForTransfer=true;

            transferTime.reset();
        }

        if (waitForTransfer && transferTime.milliseconds() > 300){
            colection.queueCommand(colection.drop);
            waitForTransfer=false;
            waitForDrop=true;
            gripTime.reset();
        }

        if (waitForDrop && gripTime.milliseconds() > 500){
            colection.queueCommand(colection.stow);
            waitForDrop=false;
            gripTime.reset();
        }

        if (gamepad1.dpad_left){
            colection.griperRotate.setPosition((colection.griperRotate.getPosition()*270)+15);
        }

        if (gamepad1.dpad_right){
            colection.griperRotate.setPosition((colection.griperRotate.getPosition()*270)-15);
        }

        if (gamepad1.start){
            colection.queueCommand(colection.grip);
        }

        if (gamepad1.back){
            colection.queueCommand(colection.drop);
        }

        driveBase.drivePowers(-gamepad1.right_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);
        if (colection.horizontalMotor.getCurrentPosition()< colection.maxSlideExtension) {
            colection.horizontalMotor.setPower(-gamepad1.left_stick_y/2);
        }

        if (delivery.slideMotor.getCurrentPosition() < delivery.maxSlideHeight) {
            double motorPower = (gamepad1.right_trigger-gamepad1.left_trigger/4);
            delivery.slideMotor.setPower(motorPower);
//            System.out.println("(gamepad1.right_trigger-gamepad1.left_trigger/4)" + motorPower);
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
//            delivery.slideMotor.setPower(0.36);

        }

        System.out.println("Testing");

        telemetry.addData("horPos", colection.horizontalMotor.getCurrentPosition());
        telemetry.addData("vertPos",delivery.slideMotor.getCurrentPosition());
        telemetry.addData("looptime ", loopTime);
        telemetry.update();
        }
}
