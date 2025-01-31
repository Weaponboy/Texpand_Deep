package org.firstinspires.ftc.teamcode.Auto.Sample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
@Autonomous
public class Sample_full extends OpModeEX {

    pathsManager paths = new pathsManager();

    follower follow = new follower();

    double targetHeading;

    boolean drop;
    ElapsedTime dropTimer=new ElapsedTime();
    ElapsedTime targetingTimer=new ElapsedTime();
    boolean collect = false;
    boolean autoQueued = false;
    boolean pullDownSlides = false;
    ElapsedTime dropTimerDriving = new ElapsedTime();
    boolean stop = false;

    boolean subRetry = false;
    double adjustedTarget = 0;

    int turnCounter = 0;

    boolean headingOverride = false;
    boolean collectRetry = false;
    boolean PIDToPoint = false;

    Vector2D powerPID = new Vector2D();

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

        public static autoState next(autoState current) {
            autoState[] values = autoState.values();
            int nextIndex = (current.ordinal() + 1) % values.length;
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

    public cycleState CycleState = cycleState.basketDrob;

    public autoState targetState = autoState.four;
    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public building cycleBuilt = building.notBuilt;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(326, 326)),
    };

    private final sectionBuilder[] spikeOne = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(338, 350), new Vector2D(322, 341))
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(305, 324))
    };

    private final sectionBuilder[] subCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(225, 290), new Vector2D(210, 228)),
    };

    private final sectionBuilder[] spikeDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(328, 328))
    };

    private final sectionBuilder[] spikeDepositSafe = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(360, 320), new Vector2D(315, 342)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(220, 280), new Vector2D(330, 333)),
    };

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initEX() {

        odometry.startPosition(344, 282, 270);

        paths.addNewPath("preloadPath");

        paths.buildPath(preloadPath);

        follow.setPath(paths.returnPath("preloadPath"));

        paths.addNewPath("collectSub");

        paths.buildPath(subCollect);

        follow.setPath(paths.returnPath("collectSub"));

        paths.addNewPath("dropBasket");

        paths.buildPath(subDeposit);

        paths.addNewPath("dropBasketSpike");

        paths.buildPath(spikeDeposit);

        paths.addNewPath("dropBasketSpikeSafe");

        paths.buildPath(spikeDepositSafe);

        follow.setPath(paths.returnPath("dropBasket"));

        paths.addNewPath("spikeOne");

        paths.buildPath(spikeOne);

        paths.addNewPath("spikeTwo");

        paths.buildPath(spikeTwo);

        collection.setCancelTransfer(true);

    }

    @Override
    public void loopEX() {

        if (state == autoState.preload) {

            if (built == building.notBuilt) {

                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;

                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 215;
                built = building.built;
                drop = true;
                dropTimer.reset();

                pathing = true;
                follow.setExtendoHeading(true);
                headingOverride = false;
            }

            if (delivery.slideMotor.getCurrentPosition() > 200 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                delivery.queueCommand(delivery.depositAuto);

            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(12, 12)) {
                delivery.queueCommand(delivery.depositAuto);

                pathing = false;

                drop = false;
            }

            if (follow.isFinished(13, 13) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop) {
                if (state == targetState) {
                    state = autoState.finished;
                } else {
                    state = autoState.spikeThree;
                    built = building.notBuilt;
                }

            }

        } else if (state == autoState.spikeThree) {

            if (built == building.notBuilt) {

                targetHeading = 183;

                built = building.built;
                cycleBuilt = building.notBuilt;

                CycleState = cycleState.spikeCollect;

                PIDToPoint = true;
                pathing = false;
            }

            if (PIDToPoint && collection.getSlidePositionCM() < 20 && odometry.X() > 318) {
                PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(319, 340), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
            } else {
                powerPID = new Vector2D();
            }

            if (CycleState == cycleState.spikeCollect) {

                if (cycleBuilt == building.notBuilt) {

                    cycleBuilt = building.built;
                    follow.setExtendoHeading(false);

                    collection.resetTransferCanceled();
                    collection.queueCommand(collection.collect);

                    pullDownSlides = false;
                    autoQueued = false;
                    runningSpikeVision = false;
                    headingOverride = false;
                }

                if (odometry.X() < 320 && !pullDownSlides) {
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (!autoQueued && odometry.Heading() < 186){
                    collection.setSlideTarget(25);
                }

                if (!headingAdjustment && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect && Math.abs(odometry.Heading() - targetHeading) < 5 && odometry.getXVelocity() < 10) {

                    autoQueued = true;

                    collection.angle = 90;

                    collection.queueCommand(collection.extendoTargetPoint(new Vector2D(243.5, 354)));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140) {
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (PIDToPoint) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(318, 340), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (cycleBuilt == building.notBuilt) {
                    PIDToPoint = true;
                    cycleBuilt = building.built;

                    targetHeading = 190;

                    drop = true;
                    autoQueued = false;
                }

                if (collection.getCurrentCommand() == collection.defaultCommand) {
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (collection.getCurrentCommand() == collection.defaultCommand && !autoQueued) {
                    collection.queueCommand(collection.preCollectNoWait);

                    collection.queueCommand(collection.extendoTargetPoint(new Vector2D(244.5, 328)));

                    collection.queueCommand(collection.collect);

                    autoQueued = true;
                }

                if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab) {
                    delivery.queueCommand(delivery.depositAuto);
                    state = autoState.spikeTwo;
                    built = building.notBuilt;
                }

            }

        }  else if (state == autoState.spikeTwo) {

            if (built == building.notBuilt) {

                targetHeading = 190;

                built = building.built;
                cycleBuilt = building.notBuilt;

                CycleState = cycleState.spikeCollect;
            }

            if (PIDToPoint && collection.getSlidePositionCM() < 20) {
                PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(316, 340), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
            } else {
                powerPID = new Vector2D();
            }

            if (CycleState == cycleState.spikeCollect) {

                if (cycleBuilt == building.notBuilt) {

                    cycleBuilt = building.built;
                    collection.resetTransferCanceled();

                    pullDownSlides = false;
                    autoQueued = false;
                    collect = false;
                    runningSpikeVision = false;
                    busyDetecting = false;
                    counter = 0;
                    follow.setExtendoHeading(false);

                    follow.finishPath();
                }

                if (odometry.X() < 320 && !pullDownSlides) {
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (delivery.getSlidePositionCM() < 35 && !autoQueued) {
                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                    autoQueued = true;
                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140 && collection.getSlideTarget() == 0) {
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt) {
                    targetHeading = 205;

                    cycleBuilt = building.built;
                    drop = true;
                    autoQueued = false;
                }

                if (PIDToPoint && collection.getSlidePositionCM() < 20) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(318, 336), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (collection.getCurrentCommand() == collection.defaultCommand) {
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (collection.getCurrentCommand() == collection.defaultCommand && !autoQueued) {
                    collection.queueCommand(collection.preCollectNoWait);

                    collection.queueCommand(collection.extendoTargetPoint(new Vector2D(244.5, 302.5)));

                    collection.queueCommand(collection.collect);

                    autoQueued = true;
                }

                if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab) {
                    delivery.queueCommand(delivery.depositAuto);
                    state = autoState.spikeOne;
                    built = building.notBuilt;
                }

            }

        } else if (state == autoState.spikeOne) {

            if (built == building.notBuilt) {
                built = building.built;
                cycleBuilt = building.notBuilt;
                CycleState = cycleState.spikeCollect;
            }

            if (CycleState == cycleState.spikeCollect) {

                if (cycleBuilt == building.notBuilt) {

                    targetHeading = 205;
                    follow.setExtendoHeading(false);

                    cycleBuilt = building.built;

                    PIDToPoint = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    collect = false;
                    runningSpikeVision = false;
                    busyDetecting = false;

                }

                if (PIDToPoint && collection.getSlidePositionCM() < 20) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(318, 334), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (odometry.X() < 320 && !pullDownSlides) {
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (delivery.getSlidePositionCM() < 35 && !autoQueued) {

                    autoQueued = true;

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140 && collection.getSlideTarget() == 0) {
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt) {
                    cycleBuilt = building.built;

                    drop = true;
                    autoQueued = false;
                    follow.setExtendoHeading(true);
                    headingOverride = false;
                }

                if (collection.getCurrentCommand() == collection.defaultCommand) {
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab) {
                    delivery.queueCommand(delivery.depositAuto);
                    state = autoState.one;
                    built = building.notBuilt;
                }

            }

        }else {

                if (built == building.notBuilt) {

                    CycleState = cycleState.subCollect;
                    cycleBuilt = building.notBuilt;
                    built = building.built;
                }

                subCycle();
            }

            if (state == autoState.finished) {
                requestOpModeStop();
            }

            if (pathing) {

                odometry.queueCommand(odometry.updateLineBased);
                RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());

                driveBase.queueCommand(driveBase.drivePowers(currentPower));
            } else {

                if (!headingOverride) {
                    if (Math.abs(targetHeading - odometry.Heading()) > 5) {
                        headingAdjustment = true;
                    } else {
                        headingAdjustment = false;
                    }
                } else {
                    headingAdjustment = false;
                }

                if (headingAdjustment) {
                    double error = targetHeading - odometry.Heading();

                    if (Math.abs(odometry.getXVelocity()) < 3 && Math.abs(odometry.getYVelocity()) < 3) {
                        if (error > 0) {
                            adjustedTarget += 0.4;
                        } else {
                            adjustedTarget -= 0.4;
                        }
                    } else {
                        adjustedTarget = 0;
                    }

                    driveBase.queueCommand(driveBase.drivePowers(new RobotPower(powerPID.getX(), powerPID.getY(), follow.getTurnPower(targetHeading + adjustedTarget, odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity()))));
                } else {
                    driveBase.queueCommand(driveBase.drivePowers(new RobotPower(powerPID.getX(), powerPID.getY(), 0)));
                }

            }

            telemetry.addData("Y", odometry.Y());
            telemetry.addData("Heading", odometry.Heading());
            telemetry.addData("X", odometry.X());
            telemetry.addData("Current command default? ", collection.getCurrentCommand() == collection.defaultCommand);
            telemetry.addData("Auto queued ", autoQueued);
            telemetry.addData("turnCounter ", turnCounter);
            telemetry.addData("boolean second", Math.abs(targetHeading - odometry.Heading()));
            telemetry.addData("", "");
            telemetry.addData("collect", collect);
            telemetry.addData("Busy detecting", busyDetecting);
            telemetry.addData("target Point", limelight.getTargetPoint());
            telemetry.update();
        }


    public void subCycle() {

        if (CycleState == cycleState.subCollect){

            if (cycleBuilt == building.notBuilt){

                cycleBuilt = building.built;

                follow.setPath(paths.returnPath("collectSub"));
                follow.usePathHeadings(true);

                busyDetecting = false;
                pathing = true;
                pullDownSlides = false;
                collect = false;
                headingOverride = false;
                collectRetry = false;

                collection.setCancelTransfer(false);
                collection.resetTransferCanceled();
                counter = 0;
            }

            if (odometry.X() < 300 && !pullDownSlides){
                pullDownSlides = true;
                delivery.queueCommand(delivery.cameraScan);

                if (!runningSpikeVision){
                    collection.queueCommand(collection.collect);
                    runningSpikeVision = false;
                }
            }

            if (follow.isFinished(6,6) && !busyDetecting && Math.abs(odometry.getXVelocity()) < 3 && Math.abs(odometry.getYVelocity()) < 3){

                autoQueued = false;
                pathing = false;
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

//            if (collectRetry && !headingAdjustment){
//
//                collect = false;
//                autoQueued = false;
//                pathing = false;
//                collectRetry = false;
//                headingOverride = true;
//
//                busyDetecting = false;
//
//                counter = 0;
//            }

//            if (collect && !collectRetry && collection.getCurrentCommand() == collection.getCurrentCommand() && collection.getFourBarState() == Collection.fourBar.stowed && targetHeading < 275 && !autoQueued) {
//
//                targetHeading = odometry.Heading() + 15;
//
//                headingOverride = false;
//
//                follow.setExtendoHeading(true);
//
//                collectRetry = true;
//
//            } else if (!collect && counter >= 29 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.stowed) && !collectRetry && targetHeading < 275 && !autoQueued) {
//
//                targetHeading = odometry.Heading() + 15;
//
//                headingOverride = false;
//
//                follow.setExtendoHeading(true);
//
//                collectRetry = true;
//
//            }

            if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 30 && collection.getCurrentCommand() == collection.defaultCommand){

                counter++;

                if (limelight.getTargetPoint() != null && counter > 2){

                    if (collection.getFourBarState() != Collection.fourBar.preCollect){
                        collection.queueCommand(collection.collect);
                    }

                    collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                    collect = true;

                    counter = 40;
                }

            }

//            if (!subRetry && collection.isTransferCanceled() && collection.getSlidePositionCM() > 0 && collection.getSlideTarget() > 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
//
//                collection.setSlideTarget(0);
//
//                delivery.queueCommand(delivery.cameraScan);
//
//                collect = false;
//                autoQueued = false;
//                pathing = false;
//                headingOverride = true;
//                subRetry = true;
//
//            } else if (delivery.getSlidePositionCM() > 15 && collection.isTransferCanceled() && Math.abs(delivery.slideMotor.getVelocity()) < 10) {
//                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
//
//                busyDetecting = false;
//
//                collection.resetTransferCanceled();
//            }
//
//            if (collection.isTransferCanceled() && subRetry && collection.getSlideTarget() != 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
//                CycleState = Blue_Left_cyclin.cycleState.basketDrob;
//                cycleBuilt = Blue_Left_cyclin.building.notBuilt;
//
//                delivery.setGripperState(Delivery.gripper.grab);
//
//                collection.setSlideTarget(0);
//                collection.overrideCurrent(true, collection.stow);
//            }

            if (follow.isFinished(10,10) && collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                collection.queueCommand(collection.transferAuto);

                collection.queueCommand(delivery.transfer);

                collection.queueCommand(collection.transferDropAuto);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                autoQueued = true;
            }

            if (follow.isFinished(10,10) && !collection.isTransferCanceled() && collection.getSlideTarget() == 0 && collection.getClawsState() == Collection.clawState.grab && delivery.getSlidePositionCM() < 15 && collect && autoQueued && collection.horizontalMotor.getVelocity() < -7 && collection.getSlidePositionCM() < 30) {
                CycleState = cycleState.basketDrob;
                cycleBuilt = building.notBuilt;
            }

        } else if (CycleState == cycleState.basketDrob){

            if (cycleBuilt == building.notBuilt){

                follow.setPath(paths.returnPath("dropBasket"));

                targetHeading = 210;

                follow.usePathHeadings(false);

                cycleBuilt = building.built;

                pathing = true;
                drop = true;
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && odometry.X() > 240 && drop){
                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;
            }

            if (delivery.slideMotor.getCurrentPosition() > 695 && delivery.fourbarState == Delivery.fourBarState.transfer && drop){
                delivery.queueCommand(delivery.depositAuto);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(9,9)) {

                delivery.queueCommand(delivery.depositAuto);

                pathing = false;

                drop = false;

                dropTimerDriving.reset();

            }

            if (state == targetState){

                if (!pathing && !drop){
                    delivery.queueCommand(delivery.depositAuto);
                    follow.setPath(paths.returnPath("spikeTwo"));
                    pathing = true;
                    stop = true;
                } else if (stop && delivery.getSlidePositionCM() < 10) {
                    state = autoState.finished;
                }

            }else{

                if (follow.isFinished(9, 9) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop) {
                    state = autoState.next(state);
                    built = building.notBuilt;
                }

            }

        }

    }
}