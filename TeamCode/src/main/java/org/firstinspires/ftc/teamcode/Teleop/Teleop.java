package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "Full_Teleop", group = "AAAAAAt the top")
public class Teleop extends OpModeEX {

    boolean firstDrop = true;
    boolean flipOutDepo = false;
    boolean autoDepo = false;
    boolean drive = false;
    boolean chamberCollect = false;
    boolean transfer = false;
    boolean clip_and_collect = false;
    boolean collectChamberTransfer = false;
    double slide_set_point = 30;

    int slideResetCounter = 0;

    boolean queueCollection = false;
    hangStates hangState = hangStates.waiting;

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    ElapsedTime rotateTimer = new ElapsedTime();

    enum hangStates{
        waiting,
        prepare,
        Hang,
        hold,
        abort
    }

    @Override
    public void initEX() {
        odometry.startPosition(82.5,100,0);
        collection.manualAngle = 0; 
        collection.setTransferType(Collection.tranfer.teleop);
    }

    @Override
    public void loopEX() {

        collection.setOpenWide(true);

        if((collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect) && !chamberCollect){
            driveBase.strafeExtra = 1.08;
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.25, (gamepad1.left_trigger - gamepad1.right_trigger)*0.2, -gamepad1.right_stick_x*0.38));
        }else if((collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect) && chamberCollect){
            driveBase.strafeExtra = 1.08;
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.45, (gamepad1.left_trigger - gamepad1.right_trigger)*0.3, -gamepad1.right_stick_x*0.48));
        }else {
            driveBase.strafeExtra = 1.2;
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.65, -gamepad1.right_stick_x*1));
        }

        overrideCurrent(currentGamepad1, lastGamepad1);
        toggleBreakBeams(currentGamepad2, lastGamepad2);
        toggleTransferRetry(currentGamepad2, lastGamepad2);
        hang(currentGamepad2, lastGamepad2);

        if (currentGamepad2.left_stick_y < 0){
            if (delivery.highBasket < 64){
                delivery.highBasket += 1;
            }
            delivery.slideSetPoint(delivery.highBasket);
        }

        if (currentGamepad2.left_stick_y > 0){
            delivery.highBasket -= 1;
            delivery.slideSetPoint(delivery.highBasket);
        }

        if (currentGamepad2.a && !lastGamepad2.a){
            collection.manualAngle = 90;
        }

        if (gamepad2.right_stick_y < 0){
            hang.hang1.setPosition(1);
            hang.hang2.setPosition(1);
        } else if (gamepad2.right_stick_y > 0) {
            hang.hang1.setPosition(0);
            hang.hang2.setPosition(0);
        }else if (hangState != hangStates.Hang){
            hang.hang1.setPosition(0.5);
            hang.hang2.setPosition(0.5);
        }

        /**
         * Toggles
         * */
        if (!chamberCollect && currentGamepad2.dpad_right && !lastGamepad2.dpad_right){
            chamberCollect = true;
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }else  if (chamberCollect && currentGamepad2.dpad_right && !lastGamepad2.dpad_right){
            chamberCollect = false;
            gamepad1.rumble(100);
            gamepad2.rumble(100);
        }

        if (slide_set_point == 30 && currentGamepad2.dpad_left && !lastGamepad2.dpad_left){
            slide_set_point = 48;
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }else  if (slide_set_point == 48 && currentGamepad2.dpad_left && !lastGamepad2.dpad_left){
            slide_set_point = 30;
            gamepad1.rumble(100);
            gamepad2.rumble(100);
        }

        if (currentGamepad2.dpad_down && !lastGamepad2.dpad_down && !delivery.isLowBucket()){
            delivery.setLowBucket(true);
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }else if (currentGamepad2.dpad_down && !lastGamepad2.dpad_down && delivery.isLowBucket()){
            delivery.setLowBucket(false);
            gamepad1.rumble(100);
            gamepad2.rumble(100);
        }

        if (currentGamepad1.start && !lastGamepad1.start && autoDepo){
            autoDepo = false;
            gamepad1.rumble(100);
        }else if (currentGamepad1.start && !lastGamepad1.start && !autoDepo){
            autoDepo = true;
            gamepad1.rumble(300);
        }

        /**
         * Collect code
         * */
        if ((currentGamepad1.right_bumper && !lastGamepad1.right_bumper) && !transfer){

            delivery.setSpikeTransfer(true);
            delivery.clearQueue();
            delivery.queueCommand(delivery.spike);
            collection.stopTargeting();

            if(!collection.getFourBarState().equals(Collection.fourBar.preCollect) && collection.getCurrentCommand() == collection.defaultCommand && !collection.getFourBarState().equals(Collection.fourBar.collect)){

                collection.queueCommand(collection.collect);

                if (collection.getSlidePositionCM() < 2 && !chamberCollect) {
                    collection.armEndPointIncrement(0, slide_set_point, false);
                }

                delivery.setGripperState(Delivery.gripper.drop);
                delivery.griperRotateSev.setPosition(90);

            }else if (collection.getFourBarState().equals(Collection.fourBar.preCollect)){

                if(collection.getFourBarState() == Collection.fourBar.preCollect && !chamberCollect) {
                    collection.queueCommand(collection.collect);
                    delivery.setGripperState(Delivery.gripper.drop);
                    collection.queueCommand(collection.transfer(Collection.tranfer.teleop));
                    collection.manualAngle = 0;
                }else if (collection.getFourBarState() == Collection.fourBar.preCollect && chamberCollect){
                    collection.queueCommand(collection.collect);
                    delivery.setGripperState(Delivery.gripper.drop);
                    collection.queueCommand(collection.transferNoSave(Collection.tranfer.underChamberCycle));
                }

                firstDrop = true;
            }

        }

        if ((currentGamepad2.right_trigger > 0 && (lastGamepad2.right_trigger != 0)) && collection.getFourBarState() == Collection.fourBar.stowedChamber && chamberCollect) {
            collection.queueCommand(collection.transferNoSave(Collection.tranfer.underChamberCycle));
            transfer = false;
        }

        if(((currentGamepad1.dpad_left && !lastGamepad1.dpad_left)) && delivery.fourbarState != Delivery.fourBarState.preClip){

            delivery.queueCommand(delivery.cameraScan);

            collection.queueCommand(collection.visionScan);

            collection.targetPositionManuel = new Vector2D(20, 20);
            collection.armEndPointIncrement(8, -12, false);

            limelight.setReturningData(true);
            limelight.setGettingResults(true);

        }

        if (((currentGamepad1.dpad_left && !lastGamepad1.dpad_left)) && delivery.getSlidePositionCM() > 15 && (delivery.getGripperState() != Delivery.gripper.grab || delivery.getGripperState() != Delivery.gripper.tightGrab)){
            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;
            delivery.setSpikeTransfer(true);
            delivery.clearQueue();
            delivery.queueCommand(delivery.spike);
            delivery.griperRotateSev.setPosition(90);
        }

        /**
         * Slide control
         * */

        if (currentGamepad1.left_stick_y < -0.4){
            collection.armEndPointIncrement(0, 1.2, false);
            if(collection.getCurrentCommand() == collection.defaultCommand && collection.horizontalMotor.getCurrentPosition() > 40 && firstDrop && collection.getFourBarState() != Collection.fourBar.preCollect){
                collection.queueCommand(collection.collect);
                delivery.setGripperState(Delivery.gripper.drop);
                firstDrop = false;
            }
        }else if (currentGamepad1.left_stick_y > 0.4){
            collection.armEndPointIncrement(0, -1.2, false);
        }

        if (currentGamepad1.left_stick_x < -0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(-Math.abs(currentGamepad1.left_stick_x*0.8), 0, false);
        }else if (currentGamepad1.left_stick_x > 0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(Math.abs(currentGamepad1.left_stick_x*0.8), 0, false);
        }

        /**
         * collection rotate
         * */
        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.manualAngle < 75 && (collection.getFourBarState() == Collection.fourBar.preCollect || rotateTimer.milliseconds() < 600)){
            collection.manualAngle = 90;
            collection.armEndPointIncrement(0, 0, false);
        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.manualAngle > 75 && (collection.getFourBarState() == Collection.fourBar.preCollect || rotateTimer.milliseconds() < 600)){
            collection.manualAngle = 0;
            collection.armEndPointIncrement(0, 0, false);
        }

        /**
         * Deposit code
         * */
        if (((currentGamepad1.left_bumper && !lastGamepad1.left_bumper) || autoDepo) && (delivery.getGripperState() == Delivery.gripper.grab || delivery.getGripperState() == Delivery.gripper.tightGrab) && collection.getCurrentCommand() == collection.returnDefaultCommand() && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getSlidePositionCM() < 18){
            flipOutDepo = true;
            drive = true;

            if (delivery.isLowBucket()){
                delivery.slideSetPoint(delivery.lowBasket);
            }else {
                delivery.slideSetPoint(delivery.highBasket);
            }

            delivery.slides = Delivery.slideState.moving;

            rotateTimer.reset();

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.getSlidePositionCM() > 50 && !(collection.getFourBarState()== Collection.fourBar.preCollect) || currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.getSlidePositionCM() > 15 && !(collection.getFourBarState() == Collection.fourBar.preCollect) && delivery.isLowBucket()){

            if (delivery.getGripperState() == Delivery.gripper.drop){
                slideResetCounter++;
            }

            if (slideResetCounter == 2){
                slideResetCounter = 0;
                delivery.setSpikeTransfer(false);
            }

            delivery.queueCommand(delivery.deposit);
        }

        if (flipOutDepo && delivery.getSlidePositionCM() > 25){
            delivery.queueCommand(delivery.deposit);
            delivery.griperRotateSev.setPosition(45);

            flipOutDepo = false;
        }

        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            collection.queueCommand(collection.transfer);
            firstDrop = true;
            queueCollection = false;

        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 10){

            counter++;

            if (limelight.getTargetPoint() != null && counter > 2){

                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                delivery.clearQueue();
                delivery.queueCommand(delivery.spike);
//                delivery.runReset();

                queueCollection = true;
                busyDetecting = false;

                counter = 40;
            }

        } else if (busyDetecting && clip_and_collect && counter >= 10) {
            gamepad2.rumble(500);
            busyDetecting = false;
            delivery.queueCommand(delivery.clipFront);
            collection.manualAngle = 0;
            collection.targetPositionManuel = new Vector2D(35, 20);
            collection.armEndPointIncrement(0,0, false);
            collectChamberTransfer = true;
        }

//        System.out.println("SlideMotor 1" + delivery.slideMotor.getCurrentPosition());
//        System.out.println("SlideMotor 2" + delivery.slideMotor2.getCurrentPosition());
//        System.out.println("collection 2" + collection.horizontalMotor.getCurrentPosition());
//        System.out.println("x velo" + odometry.getXVelocity());
//        System.out.println("y velo" + odometry.getYVelocity());

        System.out.println("Delivery current command default: " + (delivery.getCurrentCommand() == delivery.returnDefaultCommand()));
        System.out.println("Delivery current command spike: " + (delivery.getCurrentCommand() == delivery.spike));


        telemetry.addData("loop time ", loopTime);
        telemetry.addData("horizontal slides ", collection.getSlidePositionCM());
        telemetry.addData("vertical slides ", delivery.getSlidePositionCM());
        telemetry.addData("delivery slides reset ", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides reset ", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor collection ", collection.breakBeam.isPressed());
        telemetry.addData("claw sensor delivery ", delivery.clawSensor.isPressed());
        telemetry.addData("collection claw boolean ", collection.isOpenWide());
        telemetry.addData("claw collection state ", collection.getClawsState());
        telemetry.addData("Deposit collection state ", delivery.getGripperState());

        telemetry.addData("Hang left", hang.hang1Left.getPosition());
        telemetry.addData("Hang right", hang.hang1Right.getPosition());

        telemetry.addData("delivery default", delivery.getCurrentCommand() == delivery.returnDefaultCommand());

        if (limelight.getTargetPoint() != null){
            telemetry.addData("limelight results X", limelight.getTargetPoint().getTargetPoint().getX());
            telemetry.addData("limelight results Y", limelight.getTargetPoint().getTargetPoint().getY());
            telemetry.addData("limelight results angle", limelight.getTargetPoint().getAngle());
        }
        telemetry.update();
    }

    public void overrideCurrent(Gamepad current, Gamepad last){
        if (current.back && !last.back){
            delivery.setSpikeTransfer(true);
            delivery.clearQueue();
            delivery.queueCommand(delivery.stow);
            delivery.slideSetPoint(delivery.spikeTransferHeight);
            delivery.slides = Delivery.slideState.moving;
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
            if (!chamberCollect){
                collection.targetPositionManuel = new Vector2D(15, 20);
                collection.armEndPointIncrement(0,0,false);
            }
            firstDrop = true;
            delivery.griperRotateSev.setPosition(90);
        }
    }
    public void hang (Gamepad current, Gamepad last){
        if (current.y && !last.y){
            if (hangState == hangStates.waiting){
                hangState = hangStates.prepare;
                hang.setServoActive(false);
                delivery.slideSetPoint(62);
                delivery.slides = Delivery.slideState.moving;
                delivery.Hang.execute();
                collection.setHangHold(true);
                collection.queueCommand(collection.StowForHang);
            }else if (hangState == hangStates.prepare){
                hangState = hangStates.Hang;
                delivery.slides = Delivery.slideState.holdPosition;
                hang.queueCommand(hang.Engage);
            } else if (hangState == hangStates.Hang) {
                hangState = hangStates.hold;
            }

        }

    }

    public void toggleTransferRetry(Gamepad current, Gamepad last){
        if (current.start && !last.start){
            if (collection.isTransferRetryBoolean()){
                collection.setTransferRetryBoolean(false);
                gamepad2.rumble(100);
                gamepad1.rumble(100);
            }else {
                collection.setTransferRetryBoolean(true);
                gamepad2.rumble(500);
                gamepad1.rumble(500);
            }
        }
    }

    public void toggleBreakBeams(Gamepad current, Gamepad last){
        if (current.left_bumper && !last.left_bumper){
            if (collection.isCancelTransferActive()){
                collection.setCancelTransfer(false);
                gamepad2.rumble(100);
                gamepad1.rumble(100);
            }else {
                collection.setCancelTransfer(true);
                gamepad2.rumble(500);
                gamepad1.rumble(500);
            }
        }
    }
}