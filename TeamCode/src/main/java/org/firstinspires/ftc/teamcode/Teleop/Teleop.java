package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.command_library.Subsystems.Limelight;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "Full_Teleop", group = "AAAAAAt the top")
public class Teleop extends OpModeEX {

    boolean firstDrop = true;

    boolean flipOutDepo = false;
    boolean autoDepo = false;

    boolean drive = false;
    boolean pozSet = false;
    boolean lowBucket = false;
    boolean chamberCollect = false;
    boolean transfer = false;
    boolean clip_and_collect = false;
    boolean flipOutArm = false;
    boolean collectChamberTransfer = false;
    boolean armToVision = false;

    boolean busyClipping = false;
    boolean queueCollection = false;

    boolean resetGlobalTargetingHeading = false;
    hangStates hangState = hangStates.waiting;


    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    ElapsedTime rotateTimer = new ElapsedTime();

    enum teleopState{
        specimen,
        sample
    }
    enum hangStates{
        waiting,
        prepare,
        Hang,
        abort;

        public static hangStates next(hangStates current) {
            hangStates[] values = hangStates.values();
            int nextIndex = (current.ordinal() + 1) % values.length;
            return values[nextIndex];
        }
    }

    ElapsedTime pullUpTimer = new ElapsedTime();
    teleopState teleState = teleopState.sample;

    @Override
    public void initEX() {
        odometry.startPosition(82.5,100,0);
    }

    @Override
    public void loopEX() {

        switch (teleState){
            case sample:
                collection.setOpenWide(true);

                if(collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect){
                    driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.3, (gamepad1.left_trigger - gamepad1.right_trigger)*0.2, -gamepad1.right_stick_x*0.45));
                }else {
                    driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.65, -gamepad1.right_stick_x*1));
                }

                overrideCurrent(currentGamepad1, lastGamepad1);
                toggle(currentGamepad2, lastGamepad2);
                toggleBreakBeams(currentGamepad2, lastGamepad2);

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

                if (gamepad2.right_stick_y > 0){
                    hang.hang1.setPosition(1);
                    hang.hang2.setPosition(1);
                } else if (gamepad2.right_stick_y < 0) {
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
                }else  if (chamberCollect && currentGamepad2.dpad_right && !lastGamepad2.dpad_right){
                    chamberCollect = false;
                    gamepad1.rumble(100);
                }

                if (currentGamepad2.dpad_down && !lastGamepad2.dpad_down && !lowBucket){
                    lowBucket = true;
                    gamepad1.rumble(300);
                }else if (currentGamepad2.dpad_down && !lastGamepad2.dpad_down && lowBucket){
                    lowBucket = false;
                    gamepad1.rumble(100);
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

                    collection.stopTargeting();

                    if(!collection.getFourBarState().equals(Collection.fourBar.preCollect) && collection.getCurrentCommand() == collection.defaultCommand && !collection.getFourBarState().equals(Collection.fourBar.collect)){

                        collection.queueCommand(collection.collect);

                        if (collection.getSlidePositionCM() < 0.5 && !chamberCollect) {
                            collection.manualAngle = 0;
                            collection.armEndPointIncrement(0, 30, false);
                            //add the heading reset here
                        }

                        delivery.setGripperState(Delivery.gripper.drop);
                        delivery.overrideCurrent(true, delivery.stow);
                        delivery.griperRotateSev.setPosition(90);

                    }else if (collection.getFourBarState().equals(Collection.fourBar.preCollect)){

                        if(collection.getFourBarState() == Collection.fourBar.preCollect && !chamberCollect) {
                            collection.queueCommand(collection.collect);
                            delivery.setGripperState(Delivery.gripper.drop);
                            collection.queueCommand(collection.transfer(Collection.tranfer.normalSlam));
                        }else if (collection.getFourBarState() == Collection.fourBar.preCollect && chamberCollect){
                            collection.queueCommand(collection.collect);
                            delivery.setGripperState(Delivery.gripper.drop);
                            collection.queueCommand(collection.transferNoSave(Collection.tranfer.UnderChamberCycle));
                        }

                        firstDrop = true;
                    }

                }

                if ((currentGamepad2.right_trigger > 0 && (lastGamepad2.right_trigger != 0)) && collection.getFourBarState() == Collection.fourBar.stowedChamber && chamberCollect) {
                    collection.queueCommand(collection.transferNoSave(Collection.tranfer.UnderChamberCycle));
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

                if (((currentGamepad1.dpad_left && !lastGamepad1.dpad_left)) && delivery.getSlidePositionCM() > 15 && delivery.getGripperState() != Delivery.gripper.grab){
                    busyDetecting = true;
                    detectionTimer.reset();
                    counter = 0;
                    delivery.griperRotateSev.setPosition(90);
                }

                /**
                 * Slide control
                 * */

                if (currentGamepad1.left_stick_y < -0.4){
                    collection.armEndPointIncrement(0, 0.9, false);
                    if(collection.getCurrentCommand() == collection.defaultCommand && collection.horizontalMotor.getCurrentPosition() > 40 && firstDrop && collection.getFourBarState() != Collection.fourBar.preCollect){
                        collection.queueCommand(collection.collect);
                        delivery.setGripperState(Delivery.gripper.drop);
                        firstDrop = false;
                    }
                }else if (currentGamepad1.left_stick_y > 0.4){
                    collection.armEndPointIncrement(0, -0.9, false);
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
                if (((currentGamepad1.left_bumper && !lastGamepad1.left_bumper) || autoDepo) && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.slideMotor.getCurrentPosition() < 700 && !(collection.getFourBarState()== Collection.fourBar.preCollect)){
                    flipOutDepo = true;
                    drive = true;

                    if (lowBucket){
                        delivery.slideSetPoint(delivery.lowBasket);
                        delivery.slides = Delivery.slideState.moving;
                    }else {
                        delivery.slideSetPoint(delivery.highBasket);
                        delivery.slides = Delivery.slideState.moving;
                    }

                    rotateTimer.reset();

                }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.getSlidePositionCM() > 50 && !(collection.getFourBarState()== Collection.fourBar.preCollect)||currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.getSlidePositionCM() > 17 && !(collection.getFourBarState()== Collection.fourBar.preCollect) && lowBucket){
                    delivery.queueCommand(delivery.deposit);
                    pozSet = true;
                }

                if (flipOutDepo && delivery.getSlidePositionCM() > 15){
                    delivery.queueCommand(delivery.deposit);
                    delivery.griperRotateSev.setPosition(45);
                    flipOutDepo = false;
                }

                break;
            case specimen:
                if((collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect) && collection.getTransferType() == Collection.tranfer.specimenSampleCollect){
                    driveBase.queueCommand(driveBase.drivePowers(gamepad2.right_stick_y*0.6, (gamepad2.left_trigger - gamepad2.right_trigger)*0.3, -gamepad2.right_stick_x*0.6));
                }else if((collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect) && collection.getTransferType() == Collection.tranfer.specimen && collection.getSlidePositionCM() > 10){
                    driveBase.queueCommand(driveBase.drivePowers(gamepad2.right_stick_y*0.35, (gamepad2.left_trigger - gamepad2.right_trigger)*0.3, -gamepad2.right_stick_x*0.45));
                }else if (!busyClipping){
                    driveBase.queueCommand(driveBase.drivePowers(gamepad2.right_stick_y, (gamepad2.left_trigger - gamepad2.right_trigger)*0.65, -gamepad2.right_stick_x));
                }

                overrideCurrent(currentGamepad2, lastGamepad2);
                toggle(currentGamepad1, lastGamepad1);
                toggleBreakBeams(currentGamepad1, lastGamepad1);

                hang(currentGamepad1, lastGamepad1);

                /**
                 * Transfer, delivery and deposit modes
                 * */
                if (currentGamepad2.dpad_down && !lastGamepad2.dpad_down){

                    collection.setTransferType(Collection.tranfer.specimen);

                    delivery.overrideCurrent(true, delivery.stow);
                    delivery.runReset();

                    delivery.griperRotateSev.setPosition(90);

                    collection.manualAngle = 0;
                    collection.targetPositionManuel = new Vector2D(collection.getSlidePositionCM() + 15, 20);
                    collection.armEndPointIncrement(0, 0, false);

                    collection.setClawsState(Collection.clawState.drop);
                    if (collection.getFourBarState() != Collection.fourBar.preCollect){
                        collection.queueCommand(collection.collect);
                    }

                    busyDetecting = false;
                    collection.stopTargeting();

                    delivery.setGripperState(Delivery.gripper.drop);
                    gamepad2.rumble(300);

                    flipOutDepo = false;
                    clip_and_collect = false;

                }

                if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up){
                    collection.setTransferType(Collection.tranfer.specimenSampleCollect);
                    clip_and_collect = false;
                    gamepad2.rumble(300);
                }

                if (currentGamepad2.dpad_right && !lastGamepad2.dpad_right){
                    collection.setTransferType(Collection.tranfer.specimen);

                    delivery.overrideCurrent(true, delivery.stow);
                    delivery.runReset();

                    collection.manualAngle = 0;
                    collection.targetPositionManuel = new Vector2D(collection.getSlidePositionCM() + 15, 20);
                    collection.armEndPointIncrement(0, 0, false);

                    collection.setClawsState(Collection.clawState.drop);

                    if (collection.getFourBarState() != Collection.fourBar.preCollect){
                        collection.queueCommand(collection.collect);
                    }

                    delivery.griperRotateSev.setPosition(90);

                    busyDetecting = false;
                    collection.stopTargeting();

                    delivery.setGripperState(Delivery.gripper.drop);

                    clip_and_collect = true;
                    gamepad2.rumble(300);
                    delivery.griperRotateSev.setPosition(90);
                }

                /**
                 * Arm control
                 * */
                if (currentGamepad2.left_stick_y < -0.4){
                    collection.armEndPointIncrement(0, -currentGamepad2.left_stick_y, false);
                    if(collection.getCurrentCommand() == collection.defaultCommand && collection.horizontalMotor.getCurrentPosition() > 40 && firstDrop && collection.getFourBarState() != Collection.fourBar.preCollect){
                        collection.queueCommand(collection.collect);
                        delivery.setGripperState(Delivery.gripper.drop);
                        firstDrop = false;
                    }
                }else if (currentGamepad2.left_stick_y > 0.4){
                    collection.armEndPointIncrement(0, -currentGamepad2.left_stick_y, false);
                }else if (collection.slidesReset.isPressed()){
                }

                if (currentGamepad2.left_stick_x < -0.7 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
                    collection.armEndPointIncrement(-Math.abs(currentGamepad2.left_stick_x*0.7), 0, false);
                }else if (currentGamepad2.left_stick_x > 0.7 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
                    collection.armEndPointIncrement(Math.abs(currentGamepad2.left_stick_x*0.7), 0, false);
                }

                if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && collection.manualAngle < 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
                    collection.manualAngle = 90;
                    collection.armEndPointIncrement(0, 0, false);
                }else if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && collection.manualAngle > 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
                    collection.manualAngle = 0;
                    collection.armEndPointIncrement(0, 0, false);
                }

                if (flipOutArm && collection.getCurrentCommand() == collection.defaultCommand && !collection.isTransferCanceled()){
                    collection.queueCommand(collection.collect);

                    collection.targetPositionManuel = new Vector2D(6, 20);
                    collection.armEndPointIncrement(14, -10, false);

                    flipOutArm = false;
                } else if (flipOutArm && collection.getCurrentCommand() == collection.defaultCommand && collection.isTransferCanceled()) {
                    flipOutArm = false;
                }

                /**
                 * Collection code
                 * */
                if ((currentGamepad2.right_bumper && !lastGamepad2.right_bumper) && delivery.fourbarState != Delivery.fourBarState.preClip && delivery.getCurrentCommand() != delivery.preClipBack){

                    collection.setOpenWide(true);

                    if(!collection.getFourBarState().equals(Collection.fourBar.preCollect) && collection.getCurrentCommand() == collection.defaultCommand && !collection.getFourBarState().equals(Collection.fourBar.collect)){
                        collection.queueCommand(collection.collect);

                        delivery.griperRotateSev.setPosition(90);

                        if (collection.getTransferType() == Collection.tranfer.specimen){
                            if (collection.getSlidePositionCM() < 1) {
                                collection.manualAngle = 0;
                                collection.armEndPointIncrement(0, 40, false);
                            } else if (collection.getSlidePositionCM() < 22) {
                                collection.manualAngle = 0;
                                collection.armEndPointIncrement(0, 15, false);
                            }
                        }else {
                            if (collection.getSlidePositionCM() < 1) {
                                collection.manualAngle = 0;
                                collection.armEndPointIncrement(0, 30, false);
                            } else if (collection.getSlidePositionCM() < 22) {
                                collection.manualAngle = 0;
                                collection.armEndPointIncrement(0, 15, false);
                            }
                        }

                        delivery.setGripperState(Delivery.gripper.drop);

                        if (collection.getTransferType() != Collection.tranfer.specimenSampleCollect){
                            delivery.overrideCurrent(true, delivery.stow);
                        }

                        delivery.griperRotateSev.setPosition(90);
                    }else if (collection.getFourBarState().equals(Collection.fourBar.preCollect)){

                        if(collection.getFourBarState() == Collection.fourBar.preCollect) {
                            collection.queueCommand(collection.collect);
                            delivery.setGripperState(Delivery.gripper.drop);
                        }

                        collection.setSpikeTime(2.6);

                        if (collectChamberTransfer){
                            collection.queueCommand(collection.transferNoSave(Collection.tranfer.chamberCollect));
                        }else {
                            collection.queueCommand(collection.transfer);
                            flipOutDepo = true;

                            if (clip_and_collect){
                                flipOutArm = true;
                            }
                        }

                        if (collection.getTransferType() == Collection.tranfer.specimen){
                            delivery.overrideCurrent(true, delivery.stow);
                            delivery.runReset();
                        }

                        firstDrop = true;
                    }

                }else if ((currentGamepad2.right_bumper && !lastGamepad2.right_bumper) && delivery.getSlidePositionCM() > 5 && collection.getTransferType() == Collection.tranfer.specimen && !clip_and_collect){
                    delivery.queueCommand(delivery.clipBack);
                    delivery.queueCommand(delivery.releaseClip);
                }else if ((currentGamepad2.right_bumper && !lastGamepad2.right_bumper) && delivery.getSlidePositionCM() > 5 && collection.getTransferType() == Collection.tranfer.specimen && clip_and_collect){
                    busyDetecting = true;
                    detectionTimer.reset();
                    counter = 0;

                    limelight.setReturningData(true);
                    limelight.setGettingResults(true);

                    busyClipping = true;
                    collection.setOpenWide(false);
                }

                if (busyClipping && detectionTimer.milliseconds() < 300){
                    driveBase.drivePowers(-0.5, 0, 0);
                }else {
                    busyClipping = false;
                }

                if (collectChamberTransfer && collection.getSlideTarget() == 0){
                    collectChamberTransfer = false;
                }

                /**
                 * Vision code
                 * */
                if(((currentGamepad2.dpad_left && !lastGamepad2.dpad_left)) && delivery.getSlidePositionCM() < 15 && !clip_and_collect){

                    delivery.queueCommand(delivery.cameraScan);

                    collection.queueCommand(collection.visionScan);

                    collection.targetPositionManuel = new Vector2D(6, 20);
                    collection.armEndPointIncrement(14, -10, false);
                    collection.targetPositionManuel = new Vector2D(20, 20);

                    limelight.setReturningData(true);
                    limelight.setGettingResults(true);

                }else if (((currentGamepad2.dpad_left && !lastGamepad2.dpad_left))){

                    busyDetecting = true;
                    detectionTimer.reset();
                    counter = 0;

                    limelight.setReturningData(true);
                    limelight.setGettingResults(true);

                    collection.setOpenWide(false);
                }

                if (flipOutDepo && collection.getTransferType() == Collection.tranfer.specimen && collection.getCurrentCommand() == collection.defaultCommand && collection.slidesReset.isPressed() && delivery.fourbarState == Delivery.fourBarState.transfer){

                    if (!clip_and_collect){
                        delivery.queueCommand(delivery.preClipBack);
                        delivery.griperRotateSev.setPosition(0);
                    }else {
                        delivery.queueCommand(delivery.preClipFront);
                        delivery.griperRotateSev.setPosition(180);
                    }


                    flipOutDepo = false;
                }

                /**
                 * Dropping in obs zone
                 * */
                if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && collection.getFourBarState() == Collection.fourBar.stowedChamber){
                    collection.setSlideTarget(48);
                    collection.queueCommand(collection.observationDrop);
                    collection.manualAngle = 90;
                }else if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && collection.getSlidePositionCM() > 0 && collection.getClawsState() == Collection.clawState.grab){
                    collection.setClawsState(Collection.clawState.drop);

                    if (clip_and_collect){
                        collection.manualAngle = 90;
                        collection.targetPositionManuel = new Vector2D(collection.getSlidePositionCM() + 10, 20);
                        collection.armEndPointIncrement(0, 0, false);
                        collection.queueCommand(collection.collect);
                        delivery.griperRotateSev.setPosition(90);
                        collection.setOpenWide(true);
                    }else {
                        armToVision = true;
                        collection.targetPositionManuel = new Vector2D(20, 20);
                        collection.armEndPointIncrement(0, -4, false);
                        collection.queueCommand(collection.visionScan);
                        collection.manualAngle = 90;
                    }

                }

                if (armToVision && collection.getSlidePositionCM() < 10){
                    armToVision = false;
                    collection.armEndPointIncrement(14, -12, false);
                    collection.targetPositionManuel = new Vector2D(20, 20);
                }

                if (currentGamepad2.a && !lastGamepad2.a && delivery.griperRotateSev.getPositionDegrees() > 90){
                    delivery.griperRotateSev.setPosition(0);
                }else  if (currentGamepad2.a && !lastGamepad2.a && delivery.griperRotateSev.getPositionDegrees() < 90){
                    delivery.griperRotateSev.setPosition(180);
                }

                if (currentGamepad2.b && !lastGamepad2.b) {
                    delivery.queueCommand(delivery.clipFront);
                }

                break;
            default:
        }

        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            if (clip_and_collect && teleState == teleopState.specimen){
                collection.queueCommand(collection.transferNoSave(Collection.tranfer.chamberCollect));
            }else {
                collection.queueCommand(collection.transfer);
            }

            firstDrop = true;
            queueCollection = false;

        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 10){

            counter++;

            if (limelight.getTargetPoint() != null && counter > 2){

                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                if (collection.getTransferType() != Collection.tranfer.specimenSampleCollect){
                    if (clip_and_collect){
                        delivery.queueCommand(delivery.clipFront);
                    }else {
                        delivery.overrideCurrent(true, delivery.stow);
                        delivery.runReset();
                    }
                }

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

//        if (!resetGlobalTargetingHeading && collection.slidesReset.isPressed() && (collection.getFourBarState() != Collection.fourBar.preCollect || collection.getFourBarState() != Collection.fourBar.collect || collection.getFourBarState() != Collection.fourBar.transferringStates)){
//            resetGlobalTargetingHeading = true;
//            collection.targetPositionManuel = new Vector2D(collection.getSlidePositionCM() + 15, 20);
//            collection.armEndPointIncrement(0, 0, false);
//        }

        System.out.println("SlideMotor 1" + delivery.slideMotor.getCurrentPosition());
        System.out.println("SlideMotor 2" + delivery.slideMotor2.getCurrentPosition());
        System.out.println("collection 2" + collection.horizontalMotor.getCurrentPosition());

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("horizontal slides ", collection.getSlidePositionCM());
        telemetry.addData("vertical slides ", delivery.getSlidePositionCM());
        telemetry.addData("delivery slides reset ", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides reset ", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor collection ", collection.breakBeam.isPressed());
        telemetry.addData("claw sensor delivery ", delivery.clawSensor.isPressed());
        telemetry.addData("collection claw boolean ", collection.isOpenWide());
        telemetry.addData("claw collection state ", collection.getClawsState());
        if (limelight.getTargetPoint() != null){
            telemetry.addData("limelight results X", limelight.getTargetPoint().getTargetPoint().getX());
            telemetry.addData("limelight results Y", limelight.getTargetPoint().getTargetPoint().getY());
            telemetry.addData("limelight results angle", limelight.getTargetPoint().getAngle());
        }
        telemetry.update();
    }

    public void overrideCurrent(Gamepad current, Gamepad last){
        if (current.back && !last.back){
            delivery.overrideCurrent(true, delivery.stow);
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
            }

        }

    }

    public void toggle(Gamepad current, Gamepad last){
        if (current.right_bumper && !last.right_bumper){
            if (teleState == teleopState.sample){
                teleState = teleopState.specimen;
                limelight.setTargetColor(Limelight.color.blue);
                collection.setTransferType(Collection.tranfer.specimenSampleCollect);
            }else {
                teleState = teleopState.sample;
                limelight.setTargetColor(Limelight.color.yellow);
                collection.setTransferType(Collection.tranfer.normalSlam);
            }
            flipOutDepo = false;
            busyDetecting = false;
        }
    }


    public void toggleBreakBeams(Gamepad current, Gamepad last){
        if (current.left_bumper && !last.left_bumper){
            if (collection.isCancelTransferActive()){
                collection.setCancelTransfer(false);
            }else {
                collection.setCancelTransfer(true);
            }
        }
    }
}