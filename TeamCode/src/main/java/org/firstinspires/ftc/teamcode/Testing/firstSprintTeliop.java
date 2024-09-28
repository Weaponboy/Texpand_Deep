package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Subsystems.Colection;
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
    boolean waitForColection =false;

    ElapsedTime collectionTimer = new ElapsedTime();
    ElapsedTime stowTimer = new ElapsedTime();
    ElapsedTime transferTime = new ElapsedTime();
    ElapsedTime gripTime = new ElapsedTime();
    ElapsedTime depGripTime = new ElapsedTime();
    ElapsedTime depTakeFromTransferTime = new ElapsedTime();
    ElapsedTime depDropTime = new ElapsedTime();
    ElapsedTime colectionMoveTimer = new ElapsedTime();
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

            colBusyCollecting = true;
            collectionTimer.reset();
        }

        if (colBusyCollecting && collectionTimer.milliseconds() > 400){
            colection.queueCommand(colection.stow);
            colBusyCollecting =false;
            colWaitForStow =true;
            stowTimer.reset();
        }
        if (colWaitForStow && stowTimer.milliseconds() > 600){
            colection.queueCommand(colection.transfer);
            colWaitForStow =false;
            colWaitForTransfer =true;

            transferTime.reset();
        }

        if (colWaitForTransfer && transferTime.milliseconds() > 300){
            colection.queueCommand(colection.drop);
            colWaitForTransfer =false;
            colWaitForDrop =true;
            gripTime.reset();
        }

        if (colWaitForDrop && gripTime.milliseconds() > 500){
            colection.queueCommand(colection.stow);
            colWaitForDrop =false;
            gripTime.reset();
        }
        if (currentGamepad1.y&&lastGamepad1.y&&delivery.depositstate == Delivery.deposit.preTransFer){
            colection.queueCommand(colection.init);
            waitForColection =true;
            collectionTimer.reset();
        }
        if (waitForColection && collectionTimer.milliseconds()>150) {
            delivery.queueCommand(delivery.transfer);
            waitForColection =false;
            depWaitforGrip =true;
            depGripTime.reset();
        }

        if (depWaitforGrip && depGripTime.milliseconds()>300){
            delivery.queueCommand(delivery.postTransfer);
            depWaitforGrip =false;
            depWaitforTakeOutOfTansfer = true;
            depTakeFromTransferTime.reset();
        }
        if (currentGamepad1.y&&lastGamepad1.y&&delivery.depositstate == Delivery.deposit.postTransfer){
            delivery.queueCommand(delivery.dropOff);
            depWaitforTakeOutOfTansfer =false;
        }
        if (currentGamepad1.left_bumper&&lastGamepad1.left_bumper&&delivery.depositstate == Delivery.deposit.basket){
            delivery.queueCommand(delivery.drop);
            depWaitForDrop =true;
            depDropTime.reset();

        }
        if (depWaitForDrop&&depDropTime.milliseconds()>200){
            depWaitForDrop =false;
            delivery.queueCommand(delivery.behindTransfer);
        }

        if (gamepad1.dpad_left){
            colection.griperRotate.setPosition((colection.griperRotate.getPosition()*270)+15);
        }
        if (gamepad1.dpad_right){
            colection.griperRotate.setPosition((colection.griperRotate.getPosition()*270)-15);
        }
        if (gamepad1.dpad_up&&colection.collectionState ==Colection.fourBar.collect ){
            colection.queueCommand(colection.preCollect);
        }
        if (gamepad1.start){
            colection.queueCommand(colection.grip);
        }

        if (gamepad1.back){
            colection.queueCommand(colection.drop);
        }

        if (gamepad1.back){
            delivery.queueCommand(delivery.drop);
        }

        driveBase.drivePowers(-gamepad2.right_stick_y, gamepad2.left_stick_x,gamepad2.right_stick_x);

        if (colection.horizontalMotor.getCurrentPosition()< colection.maxSlideExtension) {
            colection.horizontalMotor.setPower(-gamepad1.right_stick_y/2);
        }

        if (gamepad1.left_stick_y > 0 && Math.abs(delivery.slideMotor.getCurrentPosition()) < delivery.maxSlideHeight){
            delivery.slideMotor.setPower(0.63+(gamepad1.left_stick_y));
        }else if(gamepad1.left_stick_y < 0 && Math.abs(delivery.slideMotor.getCurrentPosition()) > 10){
            delivery.slideMotor.setPower(0.63+(gamepad1.left_stick_y)/1.2);
        }else{
            delivery.slideHold();
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
        telemetry.addData("vertPos power",delivery.slideMotor.getPower());
        telemetry.addData("looptime ", loopTime);
        telemetry.addData("vertPos",delivery.slideMotor.getCurrentPosition());
        telemetry.addData("axon_pos",colection.linearPosition.getPosition());
        telemetry.update();
        }
}
