package org.firstinspires.ftc.teamcode.Testing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Sample.Sample_full;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

@Autonomous
public class Testing_Sub_Failsafe extends OpModeEX {

    pathsManager paths = new pathsManager();

    follower follow = new follower();

    double targetHeading = 235;

    boolean drop;
    ElapsedTime dropTimer=new ElapsedTime();
    boolean collect = false;
    boolean autoQueued = false;
    boolean pullDownSlides = false;
    ElapsedTime dropTimerDriving = new ElapsedTime();
    boolean stop = false;

    boolean subRetry = false;

    boolean headingOverride = false;
    boolean collectRetry = false;
    boolean failedDepo = false;

    public enum autoState{
        preload,
        spikeOne,
        spikeTwo,
        spikeThree,
        one,
        two,
        three,
        four,
        five,
        finished;

        public static Sample_full.autoState next(Sample_full.autoState current) {
            Sample_full.autoState[] values = Sample_full.autoState.values();
            int nextIndex = (current.ordinal() + 1) % values.length; // Wrap around to the first enum
            return values[nextIndex];
        }
    }

    public enum cycleState{
        spikeCollect,
        subCollect,
        basketDrob
    }
    public enum building{
        built,
        notBuilt
    }
    public enum targetAuto{
        preload,
        spikes,
        sub
    }

    boolean runningSpikeVision = false;

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    boolean pathing = false;
    boolean headingAdjustment = false;

    public Sample_full.cycleState CycleState = Sample_full.cycleState.subCollect;

    public Sample_full.autoState targetState = Sample_full.autoState.three;
    public Sample_full.autoState state = Sample_full.autoState.preload;
    public Sample_full.building built = Sample_full.building.notBuilt;
    public Sample_full.building cycleBuilt = Sample_full.building.notBuilt;

    @Override
    public void initEX() {
        odometry.startPosition(200, 200, 235);
    }

    @Override
    public void loopEX() {

        subCycle();

        if (!headingOverride){
            if (Math.abs(targetHeading - odometry.Heading()) > 5){
                headingAdjustment = true;
            }else {
                headingAdjustment = false;
            }
        }else {
            headingAdjustment = false;
        }

        if (headingAdjustment){
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0, follow.getTurnPower(targetHeading, odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity()))));

            telemetry.addData("power", follow.getTurnPower(targetHeading, odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity()));

        }else {
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0, 0)));
        }

        telemetry.addData("collectRetry", collectRetry);
        telemetry.addData("busy detecting", busyDetecting);
        telemetry.addData("autoQueued", autoQueued);
        telemetry.addData("counter", counter);
        telemetry.addData("headingOverride", headingOverride);
        telemetry.addData("boolean second", Math.abs(targetHeading - odometry.Heading()));
        telemetry.update();
    }

    public void subCycle () {

        if (CycleState == Sample_full.cycleState.subCollect){

            if (cycleBuilt == Sample_full.building.notBuilt){
                busyDetecting = false;
                pathing = true;
                pullDownSlides = false;
                collect = false;
                headingOverride = false;
                collectRetry = false;

                runningSpikeVision = false;

                collection.setCancelTransfer(true);
                counter = 0;

                cycleBuilt = Sample_full.building.built;

                delivery.queueCommand(delivery.cameraScan);
                collection.queueCommand(collection.collect);
            }

            if (!busyDetecting && Math.abs(odometry.getXVelocity()) < 3 && Math.abs(odometry.getYVelocity()) < 3 && delivery.getSlidePositionCM() > 15){

                autoQueued = false;
                headingOverride = true;
                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                busyDetecting = true;
                detectionTimer.reset();
                counter = 0;
            }

            if (collect && collection.getSlideTarget() != 0 && delivery.slideTarget > 15){
                delivery.overrideCurrent(true, delivery.stow);
                delivery.runReset();
            }

            if (collectRetry && !headingAdjustment){

                collect = false;
                autoQueued = false;
                pathing = false;
                collectRetry = false;
                headingOverride = true;

                busyDetecting = false;

                counter = 0;
            }

            if (collect && !collectRetry && collection.getCurrentCommand() == collection.getCurrentCommand() && collection.getFourBarState() == Collection.fourBar.stowed && targetHeading < 275 && !autoQueued) {

                targetHeading = odometry.Heading() + 20;

                headingOverride = false;

                follow.setExtendoHeading(true);

                collectRetry = true;

            } else if (!collect && counter >= 39 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.stowed) && !collectRetry && targetHeading < 275 && !autoQueued) {

                targetHeading = odometry.Heading() + 20;

                headingOverride = false;

                follow.setExtendoHeading(true);

                collectRetry = true;

            }

            if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 40 && collection.getCurrentCommand() == collection.defaultCommand){

                counter++;

//                if (!collection.sampleDetector.detections.isEmpty() && !collection.sampleDetector.isScanning() && counter > 12){
//
//                    if (collection.getFourBarState() != Collection.fourBar.preCollect){
//                        collection.queueCommand(collection.collect);
//                    }
//
//                    collection.sampleMap = collection.sampleDetector.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));
//
//                    collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));
//
//                    collect = true;
//
//                    counter = 40;
//                }

            }

            if (!subRetry && collection.isTransferCanceled() && collection.getSlidePositionCM() > 0 && collection.getSlideTarget() > 0 && collection.getFourBarState() == Collection.fourBar.preCollect){

                collection.setSlideTarget(0);

                delivery.queueCommand(delivery.cameraScan);

                collect = false;
                autoQueued = false;
                pathing = false;
                headingOverride = true;
                subRetry = true;

            } else if (delivery.getSlidePositionCM() > 15 && collection.isTransferCanceled() && Math.abs(delivery.slideMotor.getVelocity()) < 10) {
                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                busyDetecting = false;

                collection.resetTransferCanceled();
            }

            if (collection.isTransferCanceled() && subRetry && collection.getSlideTarget() != 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
                CycleState = Sample_full.cycleState.basketDrob;
                cycleBuilt = Sample_full.building.notBuilt;

                collection.setSlideTarget(0);
                collection.overrideCurrent(true, collection.stow);
            }

            if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                collection.queueCommand(collection.transferAuto);

                collection.queueCommand(delivery.transfer);

                collection.queueCommand(collection.transferDropAuto);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                autoQueued = true;
            }

        }

    }
}
