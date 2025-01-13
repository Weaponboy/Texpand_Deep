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
    boolean collect = false;
    boolean autoQueued = false;
    boolean pullDownSlides = false;
    ElapsedTime dropTimerDriving = new ElapsedTime();
    boolean stop = false;

    double vertical = 0;

    boolean headingOverride = false;
    boolean collectRetry = false;

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
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(311, 326)),
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(323, 311), new Vector2D(292, 326)),
    };

    private final sectionBuilder[] subCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(200, 295), new Vector2D(200, 240)),
    };

    private final sectionBuilder[] spikeDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(330, 330)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(327, 329)),
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

//                if (autoQueued && collection.getCurrentCommand() == collection.defaultCommand && collection.getFourBarState() == Collection.fourBar.preCollect){
//                    state = autoState.spikeTwo;
//                    built = building.notBuilt;
//                }

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
                    collection.setSlideTarget(5);

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

                if (busyDetecting && !collect && detectionTimer.milliseconds() > (50*counter) && counter < 20){
                    counter++;
                }

                if (busyDetecting && !collect){

                    if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning() && counter > 6){

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

                if (delivery.slideMotor.getCurrentPosition() > 695 && delivery.fourbarState == Delivery.fourBarState.transfer){
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

                counter++;

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("spikeOne"));

                    targetHeading = 178;

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

                    collection.queueCommand(collection.extendoTargetPoint(new Point(246, 329)));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.isTransferCanceled() && !runningSpikeVision){
                    collection.setSlideTarget(5);

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

                if (busyDetecting && !collect && detectionTimer.milliseconds() > (50*counter) && counter < 20){
                    counter++;
                }

                if (busyDetecting && !collect){

                    if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning() && counter > 6){

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

                if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                    collection.queueCommand(collection.transferAuto);

                    collection.queueCommand(collection.transferDropAuto);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                    autoQueued = true;
                }


                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140){
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
                }

                if (collection.getCurrentCommand() == collection.defaultCommand){
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 695 && delivery.fourbarState == Delivery.fourBarState.transfer){
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

                counter++;

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("spikeOne"));

                    targetHeading = 160;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    follow.setExtendoHeading(false);

                    runningSpikeVision = false;

                    collection.griperRotate.setPosition(135);
                }

                if (odometry.X() < 310 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (follow.isFinished(2,2) && !autoQueued){
                    pathing = false;
                }

                if (!pathing && !headingAdjustment && collection.horizontalMotor.getVelocity() < 5 && !autoQueued && Math.abs(odometry.Heading() - targetHeading) < 5 && collection.getFourBarState() == Collection.fourBar.preCollect && odometry.getXVelocity() < 10){

                    autoQueued = true;

                    pathing = false;

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

                    follow.setPath(paths.returnPath("dropBasketSpike"));

                    targetHeading = 225;

                    cycleBuilt =built;
                    pathing = true;
                    drop = true;
                }

                if (collection.getCurrentCommand() == collection.defaultCommand){
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 695 && delivery.fourbarState == Delivery.fourBarState.transfer){
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

        dashboardTelemetry.addData("Position", collection.getSlidePositionCM());
        dashboardTelemetry.addData("Target", collection.getSlideTarget());
        dashboardTelemetry.update();

        telemetry.addData("busy detecting", busyDetecting);
        telemetry.addData("collect collect", collect);
        telemetry.addData("counter ", counter);
        telemetry.update();

        if (pathing){

            odometry.queueCommand(odometry.updateLineBased);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            telemetry.addData("Loop time", loopTime);
            telemetry.addData("Y", odometry.Y());
            telemetry.addData("Heading", odometry.Heading());
            telemetry.addData("X", odometry.X());
            telemetry.addData("getVertical", currentPower.getVertical());
            telemetry.addData("getHorizontal", currentPower.getHorizontal());
            telemetry.addData("getPivot", currentPower.getPivot());
            telemetry.addData("busy detecting", collection.sampleSorterContour.isScanning());
            telemetry.update();

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
                driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0, follow.getTurnPower(targetHeading, odometry.Heading()))));
            }else {
                driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0, 0)));
            }

        }
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
                headingOverride = true;

                collection.setCancelTransfer(false);
            }

            if (odometry.X() < 300 && !pullDownSlides){
                pullDownSlides = true;
                delivery.queueCommand(delivery.cameraScan);
                if (!runningSpikeVision){
                    collection.queueCommand(collection.collect);
                    runningSpikeVision = false;
                }
            }

            if (follow.isFinished(6,6) && !busyDetecting && odometry.getXVelocity() < 5 && odometry.getYVelocity() < 5){

                autoQueued = false;
                pathing = false;
                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                collection.sampleSorterContour.setScanning(true);

                busyDetecting = true;
                detectionTimer.reset();
                counter = 0;
            }

            if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 30){

                counter++;

                if (!collection.sampleSorterContour.detections.isEmpty() && !collection.sampleSorterContour.isScanning()){

                    collection.sampleSorterContour.setScanning(false);
                    collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

                    collection.queueCommand(collection.autoCollectGlobal);
                    collection.setChamberCollect(false);

                    collect = true;

                    counter = 40;
                }

            }

            if (collect && collection.getSlideTarget() != 0 && delivery.slideTarget > 15){
                delivery.overrideCurrent(true, delivery.stow);
                delivery.runReset();
            }

//            else if (collect && collection.getCurrentCommand() == collection.defaultCommand && !collectRetry && collection.getSlideTarget() == 0 && collection.getFourBarState() == Collection.fourBar.stowed) {
//                targetHeading = odometry.Heading() - 8;
//
//                headingOverride = false;
//
//                follow.setExtendoHeading(true);
//
//                collectRetry = true;
//
////                collection.queueCommand(collection.collect);
//            }

//            if (collectRetry && !headingAdjustment){
//                autoQueued = false;
//                pathing = false;
//                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
//
//                collection.sampleSorterContour.setScanning(true);
//
//                busyDetecting = true;
//                detectionTimer.reset();
//                counter = 0;
//            }

            if (follow.isFinished(10,10) && collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                collection.queueCommand(collection.transferAuto);

                collection.queueCommand(delivery.transfer);

                collection.queueCommand(collection.transferDropAuto);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                autoQueued = true;
            }

            if (follow.isFinished(10,10) && collection.getClawsState() == Collection.clawState.grab && delivery.getSlidePositionCM() < 10 && collect && autoQueued && collection.horizontalMotor.getVelocity() < -20 && collection.getSlidePositionCM() < 25) {
                CycleState = cycleState.basketDrob;
                cycleBuilt = building.notBuilt;
            }

        } else if (CycleState == cycleState.basketDrob){

            if (cycleBuilt == building.notBuilt){

                follow.setPath(paths.returnPath("dropBasket"));

                targetHeading = 218;

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

                vertical = 1;

                pathing = false;

                drop = false;

                dropTimerDriving.reset();

            }

            if (state == targetState){

                if (!pathing && !drop && dropTimerDriving.milliseconds() > 500 && dropTimerDriving.milliseconds() < 600){
                    delivery.queueCommand(delivery.depositAuto);
                    follow.setPath(paths.returnPath("spikeOne"));
                    vertical = 0;
                    stop = true;
                } else if (stop && delivery.getSlidePositionCM() < 10) {
                    state = autoState.finished;
                }

            }else{

                if (follow.isFinished(9, 9) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop) {
                    state = autoState.next(state);
                    built = building.notBuilt;
//                    collection.queueCommand(collection.collect);
                }

            }

        }

    }
}