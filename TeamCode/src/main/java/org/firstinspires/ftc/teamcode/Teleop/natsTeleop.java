package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "Nats_Teleop", group = "AAAAAAt the top")
public class natsTeleop extends OpModeEX {

    boolean cameraScan = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    boolean fastTransfer = true;
    boolean queueCollection = false;

    boolean ranTransfer = false;

    boolean autoPreClip = false;
    boolean runClip = false;

    boolean pathing = false;
    double targetHeading;

    boolean visionRan = false;

    pathsManager paths = new pathsManager();
    follower follow = new follower();

    private final sectionBuilder[] subCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(235, 290), new Vector2D(200, 248)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(220, 280), new Vector2D(329, 329)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(344, 282, 270);

        paths.addNewPath("collectSub");
        paths.buildPath(subCollect);

        paths.addNewPath("dropBasket");
        paths.buildPath(subDeposit);

    }

    @Override
    public void loopEX() {

        if (!busyDetecting && pathing && gamepad1.right_stick_y == 0 && gamepad1.left_trigger == 0 &&  gamepad1.right_trigger == 0 && gamepad1.right_stick_x == 0){
            odometry.queueCommand(odometry.updateLineBased);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());

            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }else if (!busyDetecting){
            pathing = false;

            if(collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect){
                driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.5, (gamepad1.left_trigger - gamepad1.right_trigger)*0.4, -gamepad1.right_stick_x*0.5));
            }else {
                driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y * 0.9, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.6, -gamepad1.right_stick_x * 0.9));
            }
        }

        /**
         * Overwrites
         * */
        if (currentGamepad2.back && !lastGamepad2.back){
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
            collection.targetPositionManuel = new Vector2D(20, 20);

            cameraScan = false;
            ranTransfer = false;
        }

        /**
         * Collection code
         * */
        if (gamepad2.x && (collection.getFourBarState() == Collection.fourBar.transferUp || collection.getFourBarState() == Collection.fourBar.preCollect)){
            collection.setSlideTarget(50);
        }

        if (currentGamepad2.right_bumper && !lastGamepad2.right_bumper){

            delivery.setGripperState(Delivery.gripper.drop);
            delivery.overrideCurrent(true, delivery.stow);
            delivery.griperRotateSev.setPosition(0);

            collection.queueCommand(collection.collect);

            ranTransfer = false;
        }

//        if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up && fastTransfer){
//            fastTransfer = false;
//        }else if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up && !fastTransfer){
//            fastTransfer = true;
//        }

        if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up && collection.getTransferType() == Collection.tranfer.chamberCollect){
            collection.setTransferType(Collection.tranfer.normalSlam);
            gamepad2.rumble(100);
        }else if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up && collection.getTransferType() != Collection.tranfer.chamberCollect){
            collection.setTransferType(Collection.tranfer.chamberCollect);
            gamepad2.rumble(100);
        }

        if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && collection.getTransferType() != Collection.tranfer.normalSlam){
            collection.setTransferType(Collection.tranfer.normalSlam);
            gamepad1.rumble(300);
        }else if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && collection.getTransferType() != Collection.tranfer.sample){
            collection.setTransferType(Collection.tranfer.sample);
            gamepad1.rumble(300);
        }

        if (currentGamepad2.right_trigger > 0 && !(lastGamepad2.right_trigger > 0) && collection.getFourBarState() == Collection.fourBar.collect){

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));
            collection.targetPositionManuel = new Vector2D(20, 20);

            collection.queueCommand(collection.transfer);

            ranTransfer = true;
        }

        if (currentGamepad2.left_stick_y < -0.4){
            collection.armEndPointIncrement(0, -currentGamepad2.left_stick_y, false);
        }

        if (currentGamepad2.left_stick_y > 0.4){
            collection.armEndPointIncrement(0, -currentGamepad2.left_stick_y, false);
        }

        if (currentGamepad2.left_stick_x < -0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(-Math.abs(currentGamepad2.left_stick_x*0.5), 0, false);
        }else if (currentGamepad2.left_stick_x > 0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(Math.abs(currentGamepad2.left_stick_x*0.5), 0, false);
        }

        if (currentGamepad2.left_trigger > 0.1 && lastGamepad2.left_trigger < 0.1 && collection.manualAngle < 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.manualAngle = 90;
            collection.armEndPointIncrement(0, 0, false);
        }else if (currentGamepad2.left_trigger > 0.1 && lastGamepad2.left_trigger < 0.1 && collection.manualAngle > 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.manualAngle = 0;
            collection.armEndPointIncrement(0, 0, false);
        }

        if (gamepad2.dpad_left){
            hang.hang1.setPosition(1);
            hang.hang2.setPosition(1);
        }else if (gamepad2.dpad_right){
            hang.hang1.setPosition(0);
            hang.hang2.setPosition(0);
        }else{
            hang.hang1.setPosition(0.5);
            hang.hang2.setPosition(0.5);
        }

        if(currentGamepad2.b && !lastGamepad2.b){
            delivery.queueCommand(delivery.cameraScan);
            cameraScan = true;
        }

        if (cameraScan && delivery.getSlidePositionCM() > 15){
            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
        }

        if (currentGamepad2.a && !lastGamepad2.a){
            collection.setClawsState(Collection.clawState.drop);
            collection.queueCommand(collection.collect);
        }

        if (currentGamepad2.y && !lastGamepad2.y && delivery.getSlidePositionCM() > 15){
            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
            visionRan = true;
            cameraScan = false;
        }

        if (visionRan && Math.abs(odometry.getXVelocity()) < 2 && Math.abs(odometry.getYVelocity()) < 2){
            visionRan = false;

            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;
        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;

            if (limelight.getTargetPoint() != null){

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

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));

            collection.queueCommand(collection.transfer);

            queueCollection = false;
            ranTransfer = true;
        }

        if (currentGamepad1.start && lastGamepad1.start && autoPreClip){
            autoPreClip = false;
            runClip = false;
            gamepad1.rumble(300);
        }else if (currentGamepad1.start && lastGamepad1.start && !autoPreClip){
            autoPreClip = true;
            runClip = false;
            gamepad1.rumble(300);
        }

        /**
         * Delivery code
         * */
//        if (ranTransfer && !ranPreClip && autoPreClip && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed() && collection.getCurrentCommand() == collection.defaultCommand){
//            delivery.queueCommand(delivery.preClipFront);
//            delivery.griperRotateSev.setPosition(90);
//
//            ranPreClip = true;
//            ranTransfer = false;
//        }

        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed() && collection.getCurrentCommand() == collection.defaultCommand){
            delivery.queueCommand(delivery.preClipBack);
            delivery.griperRotateSev.setPosition(90);
        }else if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() > 100 && !runClip){
            delivery.queueCommand(delivery.clipBack);
            delivery.queueCommand(delivery.releaseClip);
//            runClip = true;
        }

//        else if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() > 100){
//            delivery.queueCommand(delivery.releaseClip);
//            runClip = false;
//        }

//        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed() && collection.getCurrentCommand() == collection.defaultCommand){
//            delivery.queueCommand(delivery.preClipFront);
//            delivery.griperRotateSev.setPosition(90);
//        }else if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() > 100){
//            delivery.queueCommand(delivery.clipFront);
//            ranPreClip = false;
//        }

        if (currentGamepad1.x && !lastGamepad1.x){
            follow.setPath(paths.returnPath("dropBasket"));
            follow.usePathHeadings(false);
            targetHeading = 225;
            pathing = true;
        }

        if (currentGamepad1.a && !lastGamepad1.a){
            follow.setPath(paths.returnPath("collectSub"));
            follow.usePathHeadings(true);
            pathing = true;
        }

        if ((currentGamepad2.left_stick_button && !(lastGamepad2.left_stick_button)) && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){

            if(collection.getFourBarState() == Collection.fourBar.preCollect) {
                collection.queueCommand(collection.collect);
                delivery.setGripperState(Delivery.gripper.drop);
            }

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));

            collection.queueCommand(collection.transfer);

            ranTransfer = true;

        }

        if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.drop){

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));

            collection.queueCommand(collection.transfer);

        }else if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.getSlidePositionCM() < 50 && delivery.slideTarget != delivery.highBasket && collection.getCurrentCommand() == collection.defaultCommand){

            delivery.slideSetPoint(delivery.highBasket);
            delivery.slides = Delivery.slideState.moving;

        }else if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.getSlidePositionCM() > 50 && delivery.fourbarState == Delivery.fourBarState.transfer){

            delivery.queueCommand(delivery.deposit);
            delivery.griperRotateSev.setPosition(90);

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.slideMotor.getCurrentPosition() > 700){

            delivery.queueCommand(delivery.deposit);

        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("pre clip thing ", autoPreClip);
        telemetry.addData("ran pre clip thing ", runClip);
        telemetry.addData("ranTransfer ", ranTransfer);
        telemetry.addData("Four bar state ", collection.getFourBarState());
        telemetry.addData("horizontal slides ", collection.getSlidePositionCM());
        telemetry.addData("vertical slides 1 ", delivery.getSlidePositionCM());
        telemetry.addData("vertical slides 2 ", delivery.slideMotor2.getCurrentPosition());
        telemetry.addData("fastTransfer ", fastTransfer);
        telemetry.addData("delivery slides", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor collection", collection.breakBeam.isPressed());
        telemetry.addData("claw sensor delivery", delivery.clawSensor.isPressed());
        telemetry.addData("Resetting slides", Math.abs(collection.turretTargetPosition - collection.turretPosition.getPosition()));
        telemetry.addData("Target point", collection.getSlideTarget());
        telemetry.addData("current command ", collection.getCurrentCommand() == collection.defaultCommand);
        if (limelight.getTargetPoint() != null){
            telemetry.addData("limelight results X", limelight.getTargetPoint().getTargetPoint().getX());
            telemetry.addData("limelight results Y", limelight.getTargetPoint().getTargetPoint().getY());
            telemetry.addData("limelight results angle", limelight.getTargetPoint().getAngle());
        }
        telemetry.update();
    }
}