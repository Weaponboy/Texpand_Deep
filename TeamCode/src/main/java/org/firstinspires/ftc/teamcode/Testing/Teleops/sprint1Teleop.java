package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;

@TeleOp
public class sprint1Teleop extends OpModeEX {

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();
    double turnPower = 0;
    double horizontal = 0;
    double lastHeading;

    double rotateTarget = 135;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initEX() {
        FtcDashboard.getInstance().startCameraStream(collection.sampleSorter, 30);

//        delivery.Deposit.execute();
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


        /**
         * Overwrites
         * */
        if (currentGamepad1.back && !lastGamepad1.back){
            collection.overrideCurrent(true, collection.stow);
            delivery.overrideCurrent(true, delivery.stow);
        }

        /**
         * Collection code
         * */
        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper){
            collection.queueCommand(collection.collect);
        }

        if (currentGamepad1.right_stick_y < 0){
            collection.setSlideTarget(collection.getSlideTarget()+0.04);
        }else if (currentGamepad1.right_stick_y > 0){
            collection.setSlideTarget(collection.getSlideTarget()-0.04);
        }

        if (currentGamepad1.right_stick_x < 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.setRailTargetPosition(collection.getRailPosition()-0.2);
        }else if (currentGamepad1.right_stick_x > 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
            collection.setRailTargetPosition(collection.getRailPosition()+0.2);
        }

        //first statement is the normal transfer in case the auto transfer fails
        //second statement is for drop and collect at the observation zone
        if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0) && collection.getFourBarState() == Collection.fourBar.collect){
            collection.queueCommand(collection.transfer);
            collection.setChamberCollect(false);
            rotateTarget = 90;
        }else if (currentGamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0) && collection.getFourBarState() == Collection.fourBar.collectChamber){
            collection.setClawsState(Collection.clawState.drop);
            collection.setSlideTarget(collection.getSlideTarget()-12);
            collection.queueCommand(collection.autoCollectGlobal);
        }
//        if (collection.getFourBarState() == Collection.fourBar.transfering && delivery.slidesState == Delivery.slideState.holdPosition && collection.clawSensor.isPressed() && !delivery.clawSensor.isPressed() && collection.horizontalMotor.getCurrentPosition()<10 && delivery.slideMotor.getCurrentPosition()<100){
//            delivery.queueCommand(delivery.transfer);
//            collection.queueCommand(collection.transferDrop);
//        }
        

        if (gamepad1.left_stick_x > 0){
            rotateTarget += 1;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if (gamepad1.left_stick_x < 0){
            rotateTarget -= 1;
            collection.griperRotate.setPosition(rotateTarget);
        }

        if(currentGamepad1.b && !lastGamepad1.b){
            collection.setNestState(Collection.Nest.specimen);
        }

        if (currentGamepad1.y && !lastGamepad1.y) {
            collection.queueCommand(collection.autoCollectGlobal);
            collection.setChamberCollect(true);
        }

        if (currentGamepad1.a && !lastGamepad1.a){
            collection.queueCommand(collection.autoCollectGlobal);
            collection.setChamberCollect(false);
        }


        /**
         * Delivery code
         * */
        if (currentGamepad2.x && !lastGamepad2.x && delivery.slideMotor.getCurrentPosition() < 200){
            delivery.queueCommand(delivery.slideSetPonts(delivery.highChamber));
        }else if (currentGamepad2.x && !lastGamepad2.x && delivery.slideMotor.getCurrentPosition() > 200){
            delivery.queueCommand(delivery.Clip);
        }

        if (currentGamepad1.left_trigger > 0 && !(lastGamepad1.left_trigger > 0)){
            delivery.queueCommand(delivery.deposit);
        }

        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper &&delivery.fourbarState == Delivery.fourBarState.behindNest){
            delivery.queueCommand(delivery.transfer);
        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && delivery.fourbarState == Delivery.fourBarState.grabNest && delivery.getGripperState() == Delivery.gripper.grab && delivery.slideMotor.getCurrentPosition() < 1000){
            delivery.queueCommand(delivery.slideSetPonts(delivery.highBasket));
        }

//        if (!collection.nestSensor.isPressed() && delivery.slidesReset.isPressed() && collection.getCurrentCommand() != collection.transfer && collection.slidesReset.isPressed() && delivery.fourbarState == Delivery.fourBarState.behindNest){
//            delivery.queueCommand(delivery.transfer);
//        }

//        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.getClawsState() == Collection.clawState.drop){
//            collection.setClawsState(Collection.clawState.grab);
//        }else if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper && collection.getClawsState() == Collection.clawState.grab){
//            collection.setClawsState(Collection.clawState.drop);
//        }

        telemetry.addData("loop time ", loopTime);
        telemetry.addData("rail position ", collection.getRailPosition());
        telemetry.addData("horizontal slides ", collection.horizontalMotor.getCurrentPosition()/(440/35));
        telemetry.addData("vertical slides ", delivery.slideMotor.getCurrentPosition()/(2250/65));
        telemetry.addData("collection  slides velo ", collection.horizontalMotor.getVelocity());
        telemetry.addData("delivery slides", delivery.slidesReset.isPressed());
        telemetry.addData("collection  slides", collection.slidesReset.isPressed());
        telemetry.addData("claw sensor", collection.clawSensor.isPressed());
        telemetry.update();
    }
}