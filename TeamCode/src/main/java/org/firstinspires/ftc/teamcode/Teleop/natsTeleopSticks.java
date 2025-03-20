package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "Stick_Teleop", group = "AAAAAAt the top")
public class natsTeleopSticks extends OpModeEX {

    ElapsedTime detectionTimer = new ElapsedTime();

    boolean cameraScan = false;
    boolean busyDetecting = false;
    boolean clipping = false;
    boolean queueCollection = false;
    boolean ranTransfer = false;
    boolean autoPreClip = false;
    boolean runClip = false;
    boolean flipOutDepo = false;
    boolean visionRan = false;

    enum hangStates{
        waiting,
        prepare,
        engage,
        attach,
        pullUp;

        public static hangStates next(hangStates current) {
            hangStates[] values = hangStates.values();
            int nextIndex = (current.ordinal() + 1) % values.length;
            return values[nextIndex];
        }
    }

    hangStates hangState = hangStates.waiting;

    ElapsedTime pullUpTimer = new ElapsedTime();
    int counter = 0;

    boolean disableManuelServos = false;

    @Override
    public void initEX() {
        odometry.startPosition(82.5, 100, 0);
    }

    @Override
    public void loopEX() {

        if(collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect){
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y*0.5, -gamepad1.left_stick_x*0.4, -gamepad1.right_stick_x*0.5));
        }else {
            driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y * 0.9, -gamepad1.left_stick_x*0.6, -gamepad1.right_stick_x * 0.9));
        }

        if (currentGamepad1.y && !lastGamepad1.y){
            hangState = hangStates.next(hangState);

            if (hangState == hangStates.attach){
                pullUpTimer.reset();
            }
        }

        switch (hangState){
            case waiting:
                break;
            case prepare:
                hang.setServoActive(false);
                delivery.slideSetPoint(62);
                delivery.slides = Delivery.slideState.moving;
                delivery.Hang.execute();
                collection.setHangHold(true);
                collection.queueCommand(collection.StowForHang);
                break;
            case engage:
                if (!hang.getServoActive()){
                    hang.queueCommand(hang.Engage);
                    hang.setServoActive(true);
                }
                break;
            case attach:
                if (pullUpTimer.milliseconds() < 200){
                    hang.hangPower.update(-0.5);
                }else {
                    hang.setServoActive(false);
                }
                delivery.setSlideDisabledForHang(true);
                limelight.shutDown();
                break;
            case pullUp:
                if (delivery.getSlidePositionCM() > 1){
                    hang.hangPower.update(-1);
                    hang.hangPower2.update(-0.2);
                    hang.hangPower3.update(-0.2);
                }else {
                    hang.pullUp(delivery.getSlidePositionCM());
                }
                break;
            default:
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
            delivery.griperRotateSev.setPosition(90);

            cameraScan = false;
            ranTransfer = false;
        }

        /**
         * Collection code
         * */
        if (currentGamepad2.right_bumper && !lastGamepad2.right_bumper){
            collection.queueCommand(collection.collect);

        }

        if (currentGamepad2.right_trigger > 0 && !(lastGamepad2.right_trigger > 0) && collection.getFourBarState() == Collection.fourBar.preCollect){

            collection.queueCommand(collection.collect);

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));
            collection.setSpikeTime(2.4);

            collection.queueCommand(collection.transfer(Collection.tranfer.spike));

            ranTransfer = true;
        }

        /**
         * Transfer toggles
         * */
        if (currentGamepad1.dpad_left && !lastGamepad1.dpad_left && clipping){
            clipping = false;
            collection.setTransferType(Collection.tranfer.normalSlam);
            gamepad1.rumble(300);
        }else if (currentGamepad1.dpad_left && !lastGamepad1.dpad_left && !clipping){
            clipping = true;
            collection.setTransferType(Collection.tranfer.normalSlam);
            gamepad1.rumble(300);
        }

        if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up && collection.getTransferType() == Collection.tranfer.chamberCollect){
            collection.setTransferType(Collection.tranfer.normalSlam);
            gamepad2.rumble(300);
        }else if (currentGamepad2.dpad_up && !lastGamepad2.dpad_up && collection.getTransferType() != Collection.tranfer.chamberCollect){
            collection.setTransferType(Collection.tranfer.chamberCollect);
            gamepad2.rumble(300);
        }

        if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && collection.getTransferType() != Collection.tranfer.normalSlam){
            collection.setTransferType(Collection.tranfer.normalSlam);
            gamepad1.rumble(300);
        }else if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down && collection.getTransferType() != Collection.tranfer.sample){
            collection.setTransferType(Collection.tranfer.sample);
            gamepad1.rumble(300);
        }

        /**
         * slide manuel control
         * */
        if (currentGamepad2.left_stick_y < -0.4){
            collection.armEndPointIncrement(0, -currentGamepad2.left_stick_y, false);
        }else if (currentGamepad2.left_stick_y > 0.4){
            collection.armEndPointIncrement(0, -currentGamepad2.left_stick_y, false);
        }

        if (currentGamepad2.left_stick_x < -0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(-Math.abs(currentGamepad2.left_stick_x*0.5), 0, false);
        }else if (currentGamepad2.left_stick_x > 0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(Math.abs(currentGamepad2.left_stick_x*0.5), 0, false);
        }

        /**Gripper angle toggle*/
        if (currentGamepad2.left_trigger > 0.1 && lastGamepad2.left_trigger < 0.1 && collection.manualAngle < 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.manualAngle = 90;
            collection.armEndPointIncrement(0, 0, false);
        }else if (currentGamepad2.left_trigger > 0.1 && lastGamepad2.left_trigger < 0.1 && collection.manualAngle > 75 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.manualAngle = 0;
            collection.armEndPointIncrement(0, 0, false);
        }

        /**
         * Hang control
         * */
        if (gamepad2.dpad_left){
            hang.hang1.setPosition(1);
            hang.hang2.setPosition(1);
        }else if (gamepad2.dpad_right){
            hang.hang1.setPosition(0);
            hang.hang2.setPosition(0);
        }else if (!hang.getServoActive() || disableManuelServos){
            hang.hang1.setPosition(0.5);
            hang.hang2.setPosition(0.5);
        }

        /**
         * Vision control
         * */
        if(currentGamepad2.b && !lastGamepad2.b){
            delivery.queueCommand(delivery.cameraScan);
            cameraScan = true;

            limelight.setReturningData(true);
        }

        if (currentGamepad2.y && !lastGamepad2.y && delivery.getSlidePositionCM() > 15){

            delivery.queueCommand(delivery.cameraScan);
            collection.queueCommand(collection.visionScan);

            collection.targetPositionManuel = new Vector2D(20, 20);
            collection.armEndPointIncrement(14, -4, false);

            limelight.setReturningData(true);
            limelight.setGettingResults(true);

            visionRan = true;

        }

        if (visionRan ){
            visionRan = false;

            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;
        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;
            System.out.println("Velocity" + (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())));
            System.out.println("Target point"+limelight.getTargetPoint());

            if (limelight.getTargetPoint() != null && counter > 4){

                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                delivery.overrideCurrent(true, delivery.stow);
                delivery.runReset();
                delivery.griperRotateSev.setPosition(90);

                limelight.setReturningData(false);

                queueCollection = true;
                busyDetecting = false;

                counter = 40;
            }

        } else if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter >= 19) {
            delivery.overrideCurrent(true, delivery.stow);
            delivery.runReset();

            busyDetecting = false;
        }

        /**Collection queueing for the vision collect*/
        if (queueCollection && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.collect){

            delivery.queueCommand(delivery.transferHold(() -> collection.getCurrentCommand() == collection.returnDefaultCommand()));

            collection.queueCommand(collection.transfer);

            queueCollection = false;

            ranTransfer = true;
        }

        /**
         * Delivery code
         * */

        /**Clipping normal*/
        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() < 100 && collection.slidesReset.isPressed() && collection.getCurrentCommand() == collection.defaultCommand){
            delivery.queueCommand(delivery.preClipBack);
            delivery.griperRotateSev.setPosition(10);
        }else if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper && delivery.slideMotor.getCurrentPosition() > 100){
            delivery.queueCommand(delivery.clipBack);
            delivery.queueCommand(delivery.releaseClip);
        }

        /**Sample delivery*/
        if (currentGamepad2.left_bumper && !lastGamepad2.left_bumper && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.slideMotor.getCurrentPosition() < 700 && !(collection.getFourBarState()== Collection.fourBar.preCollect)){

            delivery.slideSetPoint(delivery.highBasket);
            delivery.slides = Delivery.slideState.moving;
            flipOutDepo = true;

        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.getSlidePositionCM() > 50 && !(collection.getFourBarState() == Collection.fourBar.preCollect) && delivery.slideTarget == delivery.highBasket){
            delivery.queueCommand(delivery.deposit);
        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.getSlidePositionCM() > 15 && !(collection.getFourBarState() == Collection.fourBar.preCollect) && delivery.slideTarget == delivery.lowBasket){
            delivery.queueCommand(delivery.deposit);
        }

        if (flipOutDepo && delivery.getSlidePositionCM() > 15){
            delivery.queueCommand(delivery.deposit);
            flipOutDepo = false;
        }

        if (currentGamepad2.a && !lastGamepad2.a && delivery.fourbarState == Delivery.fourBarState.transfer && delivery.getGripperState() == Delivery.gripper.grab && delivery.slideMotor.getCurrentPosition() < 700 && !(collection.getFourBarState()== Collection.fourBar.preCollect)){
            delivery.slideSetPoint(delivery.lowBasket);
            delivery.slides = Delivery.slideState.moving;
            flipOutDepo = true;
        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("X", odometry.X());
        telemetry.addData("Y", odometry.Y());
        telemetry.addData("Heading", odometry.Heading());
        telemetry.addData("pre clip thing ", autoPreClip);
        telemetry.addData("ran pre clip thing ", runClip);
        telemetry.addData("ranTransfer ", ranTransfer);
        telemetry.addData("Four bar state ", collection.getFourBarState());
        telemetry.addData("horizontal slides ", collection.getSlidePositionCM());
        telemetry.addData("vertical slides 1 ", delivery.getSlidePositionCM());
        telemetry.addData("vertical slides 2 ", delivery.slideMotor2.getCurrentPosition());
        telemetry.addData("fastTransfer ", clipping);
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