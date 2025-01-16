package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Point;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
@Autonomous
public class Blue_Left_cyclin extends OpModeEX {

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

    int turnCounter = 0;

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

        public static autoState next(autoState current) {
            autoState[] values = autoState.values();
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

    public cycleState CycleState = cycleState.basketDrob;

    public autoState targetState = autoState.three;
    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public building cycleBuilt = building.notBuilt;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(328, 328)),
    };

    private final sectionBuilder[] spikeOne = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(311, 326))
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(305, 324))
    };

    private final sectionBuilder[] subCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(234, 285), new Vector2D(205, 240)),
    };

    private final sectionBuilder[] spikeDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(328, 328)),
    };

    private final sectionBuilder[] spikeDepositSafe = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(322, 322)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(332, 334)),
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

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        collection.sampleSorterContour.closestFirst = true;

        collection.setCancelTransfer(true);

    }

    @Override
    public void loopEX() {

//        if (collection.getFourBarState() == Collection.fourBar.collect && !collection.clawSensor.isPressed() && collection.horizontalMotor.getVelocity() < 10){
//            collection.queueCommand(collection.transfer);
//
//            collection.queueCommand(collection.transferDrop);
//
//            collection.queueCommand(delivery.closeGripper);
//
//            collection.queueCommand(collection.openGripper);
//        }

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

            if (delivery.slideMotor.getCurrentPosition() > 200 && delivery.fourbarState == Delivery.fourBarState.transfer){
                delivery.queueCommand(delivery.depositAuto);
//                collection.setSlideTarget(40);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(12,12)) {
                delivery.queueCommand(delivery.depositAuto);

                pathing = false;

                drop = false;
            }

            if (follow.isFinished(13, 13) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop) {
                if (state == targetState){
                    state = autoState.finished;
                }else{
                    state = autoState.spikeOne;
                    collection.setSlideTarget(20);
                    built = building.notBuilt;
                }

            }

        }else if (state == autoState.spikeOne) {

            if (built == building.notBuilt) {

                built = building.built;
                cycleBuilt = building.notBuilt;
                CycleState = cycleState.spikeCollect;
                pathing = true;
            }

            if (CycleState == cycleState.spikeCollect){

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("spikeOne"));

                    targetHeading = 200;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    collect = false;
                    runningSpikeVision = false;
                    busyDetecting = false;
                    follow.setExtendoHeading(false);
                }

                if (odometry.X() < 310 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (follow.isFinished(5,5)){
                    pathing = false;
                }

                if (!pathing && !headingAdjustment && collection.horizontalMotor.getVelocity() < 5 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect && Math.abs(odometry.Heading() - targetHeading) < 5 && odometry.getXVelocity() < 10 && !runningSpikeVision){

                    autoQueued = true;

                    pathing = false;

                    collection.queueCommand(collection.extendoTargetPoint(new Point(247, 304)));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.isTransferCanceled() && !runningSpikeVision){
                    collection.setSlideTarget(0);

                    delivery.queueCommand(delivery.cameraScan);

                    runningSpikeVision = true;

                    busyDetecting = false;
                }

                if (!collect && !busyDetecting && odometry.getXVelocity() < 5 && odometry.getYVelocity() < 5 && runningSpikeVision && delivery.getCurrentCommand() == delivery.returnDefaultCommand() && delivery.slideMotor.getVelocity() < 10){

                    autoQueued = false;
                    pathing = false;
                    delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                    collection.sampleSorterContour.setScanning(true, 5);

                    busyDetecting = true;
                    detectionTimer.reset();
                    counter = 0;
                }

                if (busyDetecting && !collect && detectionTimer.milliseconds() > (50*counter) && counter < 40){
                    counter++;
                }

                if (busyDetecting && !collect){

                    if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning() && counter > 16){

                        collection.sampleSorterContour.setScanning(false);
                        collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

                        collection.queueCommand(collection.autoCollectGlobal);
                        collection.setChamberCollect(false);

                        delivery.overrideCurrent(true, delivery.stow);
                        delivery.runReset();

                        collect = true;
                        busyDetecting = false;
                    }

                }

                if (busyDetecting && runningSpikeVision && counter >= 39){
                    collection.setSlideTarget(15);

                    delivery.overrideCurrent(true, delivery.stow);
                    delivery.runReset();

                    state = autoState.spikeTwo;
                    built = building.notBuilt;

                    failedDepo = true;
                }

                if ((collect && collection.getCurrentCommand() == collection.returnDefaultCommand() && collection.getFourBarState() == Collection.fourBar.stowed) || (autoQueued && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.preCollect && counter > 16)){
                    state = autoState.spikeTwo;
                    built = building.notBuilt;

                    failedDepo = true;
                }

                if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                    autoQueued = true;
                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140 && collection.getSlideTarget() == 0){
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("dropBasketSpike"));

                    targetHeading = 225;

                    cycleBuilt = building.built;
                    pathing = true;
                    drop = true;
                    follow.setExtendoHeading(true);
                    headingOverride = false;
                }

                if (collection.getCurrentCommand() == collection.defaultCommand){
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer){
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (follow.isFinished(6,6) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.depositAuto);
                    collection.setSlideTarget(15);
                    state = autoState.spikeTwo;
                    built = building.notBuilt;
                }

            }


        }else if (state == autoState.spikeTwo) {

            if (built == building.notBuilt) {

                built = building.built;
                cycleBuilt = building.notBuilt;

                CycleState = cycleState.spikeCollect;

                pathing = true;
            }

            if (CycleState == cycleState.spikeCollect){

                if (cycleBuilt == building.notBuilt){

                    if (failedDepo){
                        follow.setPath(paths.returnPath("spikeTwo"));
                        failedDepo = false;
                    }else {
                        follow.setPath(paths.returnPath("spikeOne"));
                    }

                    targetHeading = 178;

                    cycleBuilt = building.built;

                    if (collection.getFourBarState() != Collection.fourBar.preCollect){
                        collection.queueCommand(collection.collect);
                    }

                    collection.resetTransferCanceled();

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    collect = false;
                    runningSpikeVision = false;
                    busyDetecting = false;
                    counter = 0;
                    follow.setExtendoHeading(false);
                }

                if (odometry.X() < 320 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (follow.isFinished(3,3)){
                    pathing = false;
                }

                if (!pathing && !headingAdjustment && collection.horizontalMotor.getVelocity() < 5 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect && Math.abs(odometry.Heading() - targetHeading) < 5 && odometry.getXVelocity() < 10 && !runningSpikeVision){

                    autoQueued = true;

                    pathing = false;

                    headingOverride = true;

                    collection.queueCommand(collection.extendoTargetPoint(new Point(246, 329)));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.isTransferCanceled() && !runningSpikeVision){
                    collection.setSlideTarget(0);

                    delivery.queueCommand(delivery.cameraScan);

                    runningSpikeVision = true;

                    busyDetecting = false;
                }

                if (!collect && !busyDetecting && odometry.getXVelocity() < 5 && odometry.getYVelocity() < 5 && runningSpikeVision && delivery.getCurrentCommand() == delivery.returnDefaultCommand() && delivery.slideMotor.getVelocity() < 10){

                    autoQueued = false;
                    pathing = false;
                    delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                    collection.sampleSorterContour.setScanning(true, 5);

                    busyDetecting = true;
                    detectionTimer.reset();
                    counter = 0;
                }

                if (busyDetecting && !collect && detectionTimer.milliseconds() > (50*counter) && counter < 40){
                    counter++;
                }

                if ((autoQueued && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.preCollect && counter > 16)){
                    state = autoState.spikeThree;
                    built = building.notBuilt;

                    failedDepo = true;
                } else if (collect && collection.getCurrentCommand() == collection.returnDefaultCommand() && collection.getFourBarState() == Collection.fourBar.stowed) {
                    state = autoState.spikeThree;
                    built = building.notBuilt;

                    failedDepo = true;
                }

                if (busyDetecting && !collect){

                    if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning() && counter > 14){

                        collection.sampleSorterContour.setScanning(false);
                        collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

                        collection.queueCommand(collection.autoCollectGlobal);
                        collection.setChamberCollect(false);

                        delivery.overrideCurrent(true, delivery.stow);
                        delivery.runReset();

                        collect = true;
                        busyDetecting = false;
                    }

                }

                if (busyDetecting && runningSpikeVision && counter >= 39){
                    collection.setSlideTarget(15);

                    delivery.overrideCurrent(true, delivery.stow);
                    delivery.runReset();

                    state = autoState.spikeThree;
                    built = building.notBuilt;

                    failedDepo = true;
                }

                if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                    autoQueued = true;
                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140  && collection.getSlideTarget() == 0){
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt){

                    if (failedDepo){
                        follow.setPath(paths.returnPath("dropBasketSpikeSafe"));
                        failedDepo = false;
                    }else {
                        follow.setPath(paths.returnPath("dropBasketSpike"));
                    }

                    targetHeading = 225;

                    cycleBuilt = building.built;
                    pathing = true;
                    drop = true;
                }

                if (collection.getCurrentCommand() == collection.defaultCommand){
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer){
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (follow.isFinished(6,6) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.depositAuto);
                    collection.setSlideTarget(20);
                    state = autoState.spikeThree;
                    built = building.notBuilt;
                }

            }

        }else if (state == autoState.spikeThree) {

            if (built == building.notBuilt) {

                built = building.built;
                cycleBuilt = building.notBuilt;

                CycleState = cycleState.spikeCollect;

                pathing = true;
            }


            if (CycleState == cycleState.spikeCollect){

                if (cycleBuilt == building.notBuilt){

                    if (failedDepo){
                        follow.setPath(paths.returnPath("spikeTwo"));
                        targetHeading = 158;
                    }else {
                        follow.setPath(paths.returnPath("spikeOne"));
                        targetHeading = 159;
                    }

                    cycleBuilt = building.built;

                    if (collection.getFourBarState() != Collection.fourBar.preCollect){
                        collection.queueCommand(collection.collect);
                    }

                    collection.resetTransferCanceled();

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    follow.setExtendoHeading(false);

                    runningSpikeVision = false;
                    headingOverride = false;

                    collection.griperRotate.setPosition(135);

                    turnCounter = 0;
                }

                if (odometry.X() < 310 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (follow.isFinished(2,2) && !autoQueued){
                    pathing = false;
                }

                if (autoQueued && collection.horizontalMotor.getVelocity() < 10 && targetingTimer.milliseconds() > 200 && !headingAdjustment && turnCounter <= 2 && collection.getSlidePositionCM() < 25){
                    headingOverride = false;

                    targetHeading = odometry.Heading() - 3;

                    collection.clearQueue();

                    autoQueued = false;

                    turnCounter++;
                }

                if (!pathing && !headingAdjustment && collection.horizontalMotor.getVelocity() < 5 && !autoQueued && Math.abs(odometry.Heading() - targetHeading) < 5 && collection.getFourBarState() == Collection.fourBar.preCollect && Math.abs(odometry.getXVelocity()) < 3 && Math.abs(odometry.getYVelocity()) < 3){

                    autoQueued = true;

                    pathing = false;

                    headingOverride = true;

                    targetingTimer.reset();

                    collection.griperRotate.setPosition(135);

                    collection.queueCommand(collection.extendoTargetPoint(new Point(245, 358)));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.isTransferCanceled() && !runningSpikeVision){
                    collection.setSlideTarget(0);

                    delivery.queueCommand(delivery.cameraScan);

                    state = autoState.one;
                    built = building.notBuilt;

                    runningSpikeVision = true;
                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140){
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt){

                    if (failedDepo){
                        follow.setPath(paths.returnPath("dropBasketSpikeSafe"));
                        failedDepo = false;
                    }else {
                        follow.setPath(paths.returnPath("dropBasketSpike"));
                    }

                    targetHeading = 225;

                    cycleBuilt =built;
                    pathing = true;
                    drop = true;
                }

                if (collection.getCurrentCommand() == collection.defaultCommand){
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer){
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (follow.isFinished(6,6) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.depositAuto);
                    state = autoState.one;
                    built = building.notBuilt;
                }

            }

        } else {

            if (built == building.notBuilt){

                CycleState = cycleState.subCollect;
                cycleBuilt = building.notBuilt;
                built = building.built;
            }

            subCycle();
        }

        if (state==autoState.finished){
            requestOpModeStop();
        }

//        dashboardTelemetry.addData("Position", collection.getSlidePositionCM());
//        dashboardTelemetry.addData("Target", collection.getSlideTarget());
//        dashboardTelemetry.update();

//        telemetry.addData("busy detecting", busyDetecting);
//        telemetry.addData("collect collect", collect);
//        telemetry.addData("counter ", counter);
//        telemetry.update();

        if (pathing){

            odometry.queueCommand(odometry.updateLineBased);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());

//            telemetry.addData("Loop time", loopTime);
//            telemetry.addData("Y", odometry.Y());
//            telemetry.addData("Heading", odometry.Heading());
//            telemetry.addData("X", odometry.X());
//            telemetry.addData("getVertical", currentPower.getVertical());
//            telemetry.addData("getHorizontal", currentPower.getHorizontal());
//            telemetry.addData("getPivot", currentPower.getPivot());
//            telemetry.addData("busy detecting", collection.sampleSorterContour.isScanning());
//            telemetry.update();

            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }else{

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

        }

        System.out.println("headingI += 0.00006: " + headingAdjustment);
        System.out.println("heading error " + Math.abs(targetHeading - odometry.Heading()));

        telemetry.addData("Auto queued ", autoQueued);
        telemetry.addData("turnCounter ", turnCounter);
        telemetry.addData("headingOverride", headingOverride);
        telemetry.addData("odometry heading", odometry.Heading());
        telemetry.addData("headingAdjustment", headingAdjustment);
        telemetry.addData("boolean second", Math.abs(targetHeading - odometry.Heading()));
        telemetry.update();
    }

    public void subCycle () {

        if (CycleState == cycleState.subCollect){

            if (cycleBuilt == building.notBuilt){
                follow.setPath(paths.returnPath("collectSub"));
                follow.usePathHeadings(true);

                cycleBuilt = building.built;

                busyDetecting = false;
                pathing = true;
                pullDownSlides = false;
                collect = false;
                headingOverride = false;
                collectRetry = false;

                collection.setCancelTransfer(true);
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

                collection.sampleSorterContour.setScanning(true, 5);

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

                targetHeading = odometry.Heading() + 15;

                headingOverride = false;

                follow.setExtendoHeading(true);

                collectRetry = true;

            } else if (!collect && counter >= 29 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.stowed) && !collectRetry && targetHeading < 275 && !autoQueued) {

                targetHeading = odometry.Heading() + 15;

                headingOverride = false;

                follow.setExtendoHeading(true);

                collectRetry = true;

            }

            if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 30 && collection.getCurrentCommand() == collection.defaultCommand){

                counter++;

                if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning() && counter > 16){

                    if (collection.getFourBarState() != Collection.fourBar.preCollect){
                        collection.queueCommand(collection.collect);
                    }

                    collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

                    collection.queueCommand(collection.autoCollectGlobal);

                    collect = true;

                    counter = 40;
                }

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

                collection.sampleSorterContour.setScanning(true, 5);

                busyDetecting = false;

                collection.resetTransferCanceled();
            }

            if (collection.isTransferCanceled() && subRetry && collection.getSlideTarget() != 0 && collection.getFourBarState() == Collection.fourBar.preCollect){
                CycleState = Blue_Left_cyclin.cycleState.basketDrob;
                cycleBuilt = Blue_Left_cyclin.building.notBuilt;

                collection.setSlideTarget(0);
                collection.overrideCurrent(true, collection.stow);
            }

            if (follow.isFinished(10,10) && collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                collection.queueCommand(collection.transferAuto);

                collection.queueCommand(delivery.transfer);

                collection.queueCommand(collection.transferDropAuto);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                autoQueued = true;
            }

            if (follow.isFinished(10,10) && !collection.isTransferCanceled() && collection.getSlideTarget() == 0 && collection.getClawsState() == Collection.clawState.grab && delivery.getSlidePositionCM() < 10 && collect && autoQueued && collection.horizontalMotor.getVelocity() < -10 && collection.getSlidePositionCM() < 25) {
                CycleState = cycleState.basketDrob;
                cycleBuilt = building.notBuilt;
            }

        } else if (CycleState == cycleState.basketDrob){

            if (cycleBuilt == building.notBuilt){

                follow.setPath(paths.returnPath("dropBasket"));

                targetHeading = 216;

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

                if (!pathing && !drop && dropTimerDriving.milliseconds() > 500 && dropTimerDriving.milliseconds() < 600){
                    delivery.queueCommand(delivery.depositAuto);
                    follow.setPath(paths.returnPath("spikeOne"));
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