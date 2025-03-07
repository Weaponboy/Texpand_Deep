package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.command_library.Subsystems.Limelight;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "Specimen_Teleop", group = "AAAAAAt the top")
public class SpecimenTeleop extends OpModeEX {

    /**Booleans*/
    boolean firstDrop = true;
    boolean scanpos = false;
    boolean folowing = false;
    boolean queueCollection = false;
    boolean busyDetecting = false;
    boolean flipOutDepo = false;

    boolean slowDrive = false;

    /**Numbers**/
    double Heading;
    int counter = 0;

    /**Objects*/
    pathsManager paths = new pathsManager();
    follower follow = new follower();
    ElapsedTime detectionTimer = new ElapsedTime();

    @Override
    public void initEX() {

        odometry.startPosition(82.5,100,0);

        paths.addNewPath("DepositPath");

        collection.setTransferType(Collection.tranfer.specimenSampleCollect);

        limelight.setTargetColor(Limelight.color.red);

    }

    @Override
    public void loopEX() {

        /**
         * drive base code
         * */
        if (folowing && !(gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1)){

            RobotPower RobotPower = follow.followPathAuto(Heading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(RobotPower));

        }else {
            folowing = false;
            if((collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect) && collection.getTransferType() == Collection.tranfer.specimenSampleCollect){
                driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.6, (gamepad1.left_trigger - gamepad1.right_trigger)*0.3, -gamepad1.right_stick_x*0.6));
            }else if((collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect) && collection.getTransferType() == Collection.tranfer.specimen){
                driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.35, (gamepad1.left_trigger - gamepad1.right_trigger)*0.25, -gamepad1.right_stick_x*0.35));
            }else {
                driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.65, -gamepad1.right_stick_x));
            }
        }

        /**
         * Overwrites
         * */
        if (currentGamepad1.back && !lastGamepad1.back){
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
            scanpos = false;
            firstDrop = true;
        }

        /**
         * Collection code
         * */
        if ((currentGamepad1.right_bumper && !lastGamepad1.right_bumper) && delivery.fourbarState != Delivery.fourBarState.preClip){

            if(!collection.getFourBarState().equals(Collection.fourBar.preCollect) && collection.getCurrentCommand() == collection.defaultCommand && !collection.getFourBarState().equals(Collection.fourBar.collect)){
                collection.queueCommand(collection.collect);

                delivery.griperRotateSev.setPosition(90);

                if (collection.getTransferType() == Collection.tranfer.specimen){
                    if (collection.getSlidePositionCM() < 0.5) {
                        collection.manualAngle = 0;
                        collection.armEndPointIncrement(0, 40, false);
                    } else if (collection.getSlidePositionCM() < 22) {
                        collection.manualAngle = 0;
                        collection.armEndPointIncrement(0, 15, false);
                    }
                }else {
                    if (collection.getSlidePositionCM() < 0.5) {
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

                collection.queueCommand(collection.transfer);

                slowDrive = false;

                flipOutDepo = true;
                firstDrop = true;
            }

        }else if ((currentGamepad1.right_bumper && !lastGamepad1.right_bumper) && delivery.getSlidePositionCM() > 5 && collection.getTransferType() == Collection.tranfer.specimen){
            delivery.queueCommand(delivery.clipBack);
            delivery.queueCommand(delivery.releaseClipScan);


//
//            if (collection.getSlidePositionCM() < 0.5) {
//                collection.manualAngle = 0;
//                collection.armEndPointIncrement(0, 40, false);
//            } else if (collection.getSlidePositionCM() < 22) {
//                collection.manualAngle = 0;
//                collection.armEndPointIncrement(0, 15, false);
//            }
//
//            collection.queueCommand(collection.collect);
        }

        if (currentGamepad1.left_stick_y < -0.4){
            collection.armEndPointIncrement(0, 0.7, false);
            if(collection.getCurrentCommand() == collection.defaultCommand && collection.horizontalMotor.getCurrentPosition() > 40 && firstDrop && collection.getFourBarState() != Collection.fourBar.preCollect){
                collection.queueCommand(collection.collect);
                delivery.setGripperState(Delivery.gripper.drop);
                firstDrop = false;
            }
        }else if (currentGamepad1.left_stick_y > 0.4){
            collection.armEndPointIncrement(0, -0.7, false);
        }else if (collection.slidesReset.isPressed()){
        }

        if (currentGamepad1.dpad_up && !lastGamepad1.dpad_up && collection.getFourBarState() != Collection.fourBar.wallCollect){

            collection.queueCommand(collection.wallCollect);
            delivery.setGripperState(Delivery.gripper.drop);

        } else if (currentGamepad1.dpad_up && !lastGamepad1.dpad_up && collection.getFourBarState() == Collection.fourBar.wallCollect) {
            collection.queueCommand(collection.transferNoSave(Collection.tranfer.wallCollect));
        }

        if (currentGamepad1.left_stick_x < -0.7 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(-Math.abs(currentGamepad1.left_stick_x*0.7), 0, false);
        }else if (currentGamepad1.left_stick_x > 0.7 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(Math.abs(currentGamepad1.left_stick_x*0.7), 0, false);
        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.manualAngle < 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.manualAngle = 90;
            collection.armEndPointIncrement(0, 0, false);
        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.manualAngle > 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.manualAngle = 0;
            collection.armEndPointIncrement(0, 0, false);
        }

        /**
         * Transfer toggle
         * */
        if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && collection.getTransferType() != Collection.tranfer.specimen){
            collection.setTransferType(Collection.tranfer.specimen);
            delivery.overrideCurrent(true, delivery.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
            gamepad1.rumble(300);
            flipOutDepo = false;
        }else if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && collection.getTransferType() != Collection.tranfer.specimenSampleCollect){
            collection.setTransferType(Collection.tranfer.specimenSampleCollect);
            gamepad1.rumble(300);
        }

        /**
         * Vision
         * */
        if(((currentGamepad1.dpad_left && !lastGamepad1.dpad_left)) && delivery.getSlidePositionCM() < 15){

            delivery.queueCommand(delivery.cameraScan);

            collection.queueCommand(collection.visionScan);

            collection.targetPositionManuel = new Vector2D(6, 20);
            collection.armEndPointIncrement(14, -4, false);

            limelight.setReturningData(true);
            limelight.setGettingResults(true);

        }else if (((currentGamepad1.dpad_left && !lastGamepad1.dpad_left))){

            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;
            scanpos = false;
            delivery.griperRotateSev.setPosition(90);

        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;

            if (limelight.getTargetPoint() != null && counter > 2){

                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                if (collection.getTransferType() != Collection.tranfer.specimenSampleCollect){
                    delivery.overrideCurrent(true, delivery.stow);
                    delivery.runReset();
                }

                flipOutDepo = true;
                queueCollection = true;
                busyDetecting = false;

                counter = 40;
            }

        } else if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter > 20) {

            delivery.overrideCurrent(true, delivery.stow);
            delivery.runReset();

            busyDetecting = false;
        }

        /**
         * Collection queuing code
         * */
        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            collection.queueCommand(collection.transfer);

            queueCollection = false;
        }

        /**
         * Spec delivery code
         * */

        if (flipOutDepo && collection.getTransferType() == Collection.tranfer.specimen && collection.getCurrentCommand() == collection.defaultCommand && collection.slidesReset.isPressed() && delivery.fourbarState == Delivery.fourBarState.transfer){
            delivery.queueCommand(delivery.preClipBack);
            delivery.griperRotateSev.setPosition(10);
            flipOutDepo = false;
        }

        /**
         * Dropping in obs zone
         * */
        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.getFourBarState() == Collection.fourBar.stowedChamber){
            collection.setSlideTarget(48);
            collection.queueCommand(collection.observationDrop);
            collection.manualAngle = 90;
        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.getSlidePositionCM() > 0 && collection.getClawsState() == Collection.clawState.grab){
            collection.setClawsState(Collection.clawState.drop);
            collection.targetPositionManuel = new Vector2D(20, 20);
            collection.setSlideTarget(0);
            collection.queueCommand(collection.stow);
            collection.manualAngle = 90;
        }

        if (currentGamepad1.a && !lastGamepad1.a){
            delivery.queueCommand(delivery.preClipFront);
//            collection.queueCommand(collection.stowClipFront);
        } else if (currentGamepad1.b && !lastGamepad1.b) {
            delivery.queueCommand(delivery.clipFront);
        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("horizontal slides ", collection.horizontalMotor.getCurrentPosition());
        telemetry.addData("vertical slides ", delivery.getSlidePositionCM());
        telemetry.addData("collection current command ", collection.getCurrentCommand());
        telemetry.addData("delivery slides", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor collection", collection.breakBeam.isPressed());
        telemetry.addData("claw sensor delivery", delivery.clawSensor.isPressed());
        if (limelight.getTargetPoint() != null){
            telemetry.addData("limelight results X", limelight.getTargetPoint().getTargetPoint().getX());
            telemetry.addData("limelight results Y", limelight.getTargetPoint().getTargetPoint().getY());
            telemetry.addData("limelight results angle", limelight.getTargetPoint().getAngle());
        }
        telemetry.update();
    }
}