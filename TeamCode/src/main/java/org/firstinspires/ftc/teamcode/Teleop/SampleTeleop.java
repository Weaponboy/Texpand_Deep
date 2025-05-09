package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "Sample_Teleop", group = "AAAAAAt the top")
public class SampleTeleop extends OpModeEX {
    boolean firstDrop = true;
    boolean rotated = false;
    boolean scanpos = false;
    boolean folowing = false;
    double Horisontal;
    double Vertical;
    double Heading;
    pathsManager paths = new pathsManager();
    follower follow = new follower();
    boolean flipOutDepo = false;
    double rotateTarget = 90;
    boolean autoDepo = false;

    boolean drive = false;
    boolean pozSet = false;
    boolean lowBucket = false;
    boolean chamberColect = false;
    boolean tranfer = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();
    boolean queueCollection = false;

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;
//    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
//            () -> paths.addPoints(new Vector2D(200, 240), new Vector2D(220, 280), new Vector2D(329, 329)),
//    };
    @Override
    public void initEX() {
        odometry.startPosition(344, 282, 270);
//        paths.addNewPath("dropBasket");
//        paths.buildPath(subDeposit);

        limelight.setAuto(true);
        collection.setCancelTransfer(false);
    }

    @Override
    public void loopEX() {

        if (folowing && !(gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1)){
            RobotPower RobotPower = follow.followPathAuto(Heading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(RobotPower));

        }else {
            if(collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect){
                driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.3, (gamepad1.left_trigger - gamepad1.right_trigger)*0.2, -gamepad1.right_stick_x*0.45));
            }else {
                driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.65, -gamepad1.right_stick_x*1));
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
            collection.targetPositionManuel = new Vector2D(20, 20);
            scanpos = false;
            firstDrop = true;
        }
        /**
         * Collection code
         * */
        if (!chamberColect && currentGamepad1.dpad_right && !lastGamepad1.dpad_right){
            chamberColect = true;
        }else  if (chamberColect && currentGamepad1.dpad_right && !lastGamepad1.dpad_right){
            chamberColect = false;
        }

        if ((currentGamepad1.left_stick_button && !(lastGamepad1.left_stick_button)) && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){

            if(collection.getFourBarState() == Collection.fourBar.preCollect) {
                collection.queueCommand(collection.collect);
                delivery.setGripperState(Delivery.gripper.drop);
            }

            collection.setSpikeTime(2.5);

            delivery.setSpikeTransfer(true);

            collection.queueCommand(collection.transfer(Collection.tranfer.spike));

            firstDrop = true;

        }

        if ((currentGamepad1.right_bumper && !lastGamepad1.right_bumper) && !tranfer){

            collection.stopTargeting();

            if(!collection.getFourBarState().equals(Collection.fourBar.preCollect) && collection.getCurrentCommand() == collection.defaultCommand && !collection.getFourBarState().equals(Collection.fourBar.collect)){

                collection.queueCommand(collection.collect);

                if (collection.getSlidePositionCM() < 0.5) {
                    collection.manualAngle = 0;
                    collection.armEndPointIncrement(0, 30, false);
                }

                delivery.setGripperState(Delivery.gripper.drop);
                delivery.overrideCurrent(true, delivery.stow);
                delivery.griperRotateSev.setPosition(90);

            }else if (collection.getFourBarState().equals(Collection.fourBar.preCollect)){



                if(collection.getFourBarState() == Collection.fourBar.preCollect && !chamberColect) {
                    collection.queueCommand(collection.collect);
                    delivery.setGripperState(Delivery.gripper.drop);
                    collection.queueCommand(collection.transfer(Collection.tranfer.auto));
                }else if (collection.getFourBarState() == Collection.fourBar.preCollect && chamberColect){
//                    collection.queueCommand(collection.collect);
//                    delivery.setGripperState(Delivery.gripper.drop);
//                    collection.queueCommand(collection.transfer(Collection.tranfer.UnderChamberCycle));
//                    tranfer = true;
                }




                firstDrop = true;
            }
        } else if ((currentGamepad1.right_bumper && !lastGamepad1.right_bumper) && tranfer) {
            collection.queueCommand(collection.transfer(Collection.tranfer.UnderChamberCycle));
        }

        if (currentGamepad1.left_stick_y < -0.4){
            collection.armEndPointIncrement(0, 0.9, false);
            if(collection.getCurrentCommand() == collection.defaultCommand && collection.horizontalMotor.getCurrentPosition() > 40 && firstDrop && collection.getFourBarState() != Collection.fourBar.preCollect){
                collection.queueCommand(collection.collect);
                delivery.setGripperState(Delivery.gripper.drop);
                firstDrop = false;
            }
        }else if (currentGamepad1.left_stick_y > 0.4){
            collection.armEndPointIncrement(0, -0.9, false);
        }else if (collection.slidesReset.isPressed()){
        }

        if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && !lowBucket){
            lowBucket = true;
            gamepad1.rumble(300);
        }else if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && lowBucket){
            lowBucket = false;
            gamepad1.rumble(100);
        }

        if (currentGamepad1.left_stick_x < -0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(-Math.abs(currentGamepad1.left_stick_x*0.8), 0, false);
        }else if (currentGamepad1.left_stick_x > 0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(Math.abs(currentGamepad1.left_stick_x*0.8), 0, false);
        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.manualAngle < 75 && (collection.getFourBarState() == Collection.fourBar.preCollect||collection.getFourBarState() == Collection.fourBar.transferringStates)){
            collection.manualAngle = 90;
            collection.armEndPointIncrement(0, 0, false);
        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.manualAngle > 75 && (collection.getFourBarState() == Collection.fourBar.preCollect||collection.getFourBarState() == Collection.fourBar.transferringStates)){
            collection.manualAngle = 0;
            collection.armEndPointIncrement(0, 0, false);
        }

        if(((currentGamepad1.dpad_left && !lastGamepad1.dpad_left)) && delivery.fourbarState != Delivery.fourBarState.preClip && delivery.getGripperState() != Delivery.gripper.grab){

            delivery.queueCommand(delivery.cameraScan);

            collection.queueCommand(collection.visionScan);

            collection.targetPositionManuel = new Vector2D(20, 20);
            collection.armEndPointIncrement(14, -4, false);

            limelight.setReturningData(true);
            limelight.setGettingResults(true);

        }

        if (((currentGamepad1.dpad_left && !lastGamepad1.dpad_left)) && delivery.getSlidePositionCM() > 15 && delivery.getGripperState() != Delivery.gripper.grab){

            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;
            scanpos = false;
            delivery.griperRotateSev.setPosition(90);

        }

        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            collection.queueCommand(collection.transfer);

            firstDrop = true;
            queueCollection = false;

        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;

            if (limelight.getTargetPoint() != null && counter > 2){

                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                delivery.overrideCurrent(true, delivery.stow);
                delivery.runReset();

                queueCollection = true;
                busyDetecting = false;

                counter = 40;
            }

        } else if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter > 20) {

            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();

            busyDetecting = false;
        }

        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            collection.setSpikeTime(2);

            collection.queueCommand(collection.transfer(Collection.tranfer.normalSlam));

            queueCollection = false;
        }
        /**
         * Delivery code
         * */
        if (currentGamepad1.start && !lastGamepad1.start && autoDepo){
            autoDepo = false;
            gamepad1.rumble(100);
        }else if (currentGamepad1.start && !lastGamepad1.start && !autoDepo){
            autoDepo = true;
            gamepad1.rumble(300);
        }

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

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.getSlidePositionCM() > 50 && !(collection.getFourBarState()== Collection.fourBar.preCollect)||currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.getSlidePositionCM() > 17 && !(collection.getFourBarState()== Collection.fourBar.preCollect) && lowBucket){
            delivery.queueCommand(delivery.deposit);
            pozSet = true;
        }

        if (flipOutDepo && delivery.getSlidePositionCM() > 15){
            delivery.queueCommand(delivery.deposit);
            delivery.griperRotateSev.setPosition(45);
            flipOutDepo = false;
        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("horizontal slides ", collection.horizontalMotor.getCurrentPosition());
        telemetry.addData("vertical slides ", delivery.getSlidePositionCM());
        telemetry.addData("collection current command ", collection.getCurrentCommand());
        telemetry.addData("delivery slides", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor collection", collection.breakBeam.isPressed());
        telemetry.addData("claw sensor delivery", delivery.clawSensor.isPressed());
        telemetry.addData("slide target", collection.getSlideTarget());
        if (limelight.getTargetPoint() != null){
            telemetry.addData("limelight results X", limelight.getTargetPoint().getTargetPoint().getX());
            telemetry.addData("limelight results Y", limelight.getTargetPoint().getTargetPoint().getY());
            telemetry.addData("limelight results angle", limelight.getTargetPoint().getAngle());
        }
        telemetry.update();
    }
}