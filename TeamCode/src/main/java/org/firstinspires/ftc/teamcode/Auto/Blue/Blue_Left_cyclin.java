package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.Red.Red_Right_Full_Auto;
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

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    boolean pathing = false;

    public cycleState CycleState = cycleState.basketDrob;

    public autoState targetState = autoState.three;
    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public building cycleBuilt = building.notBuilt;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(332, 332)),
    };

    private final sectionBuilder[] spikeOne = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(311, 326)),
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(323, 311), new Vector2D(292, 326)),
    };

    private final sectionBuilder[] subCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(204, 288), new Vector2D(200, 238)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(331, 331)),
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

        follow.setPath(paths.returnPath("dropBasket"));

        paths.addNewPath("spikeOne");

        paths.buildPath(spikeOne);

        paths.addNewPath("spikeTwo");

        paths.buildPath(spikeTwo);

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        collection.sampleSorterContour.closestFirst = true;
    }

    @Override
    public void loopEX() {

        if (state == autoState.preload) {

            if (built == building.notBuilt) {

                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;

                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 235;
                built = building.built;
                drop = true;
                dropTimer.reset();

                pathing = true;
                follow.setExtendoHeading(true);
            }

            if (delivery.slideMotor.getCurrentPosition() > 200 && delivery.fourbarState == Delivery.fourBarState.transfer){
                delivery.queueCommand(delivery.deposit);
//                collection.setSlideTarget(40);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(12,12)) {
                delivery.queueCommand(delivery.deposit);

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

                counter++;

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("spikeOne"));

                    targetHeading = 203;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    follow.setExtendoHeading(false);
                }

                if (odometry.X() < 320 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.deposit);
                }

                if (!autoQueued){
                    collection.targetPointWithExtendoNoArm(new Vector2D(250,306));
                }

                Vector2D armPosition = collection.extendoPoint();

                if (Math.abs(armPosition.getX() - 249) < 20 && collection.horizontalMotor.getVelocity() < 5 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect && Math.abs(odometry.Heading() - targetHeading) < 5){

                    autoQueued = true;

                    pathing = false;

                    collection.queueCommand(collection.extendoTargetPoint(new Point(250, 306)));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(collection.transferDrop);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140){
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("dropBasket"));

                    targetHeading = 225;

                    cycleBuilt = building.built;
                    pathing = true;
                    drop = true;
                    follow.setExtendoHeading(true);
                }

                if (collection.getCurrentCommand() == collection.defaultCommand){
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 695 && delivery.fourbarState == Delivery.fourBarState.transfer){
                    delivery.queueCommand(delivery.deposit);
                }

                if (follow.isFinished(8,8) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.deposit);
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

                    targetHeading = 175;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    follow.setExtendoHeading(false);
                }

                if (odometry.X() < 320 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.deposit);
                }

                if (!autoQueued){
                    collection.targetPointWithExtendoNoArm(new Vector2D(246,329));
                }

                Vector2D armPosition = collection.extendoPoint();

                if (Math.abs(armPosition.getX() - 250) < 20 && collection.horizontalMotor.getVelocity() < 5 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect && Math.abs(odometry.Heading() - targetHeading) < 5){

                    autoQueued = true;

                    pathing = false;

                    collection.queueCommand(collection.extendoTargetPoint(new Point(246, 328.5)));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(collection.transferDrop);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140){
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("dropBasket"));

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
                    delivery.queueCommand(delivery.deposit);
                }

                if (follow.isFinished(8,8) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.deposit);
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

                    targetHeading = 162;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    follow.setExtendoHeading(false);
                }

                if (odometry.X() < 320 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.deposit);
                }

                if (!autoQueued){
                    collection.targetPointWithExtendoNoArm(new Vector2D(247,354));
                }

                Vector2D armPosition = collection.extendoPoint();

                if (Math.abs(armPosition.getX() - 250) < 10 && collection.horizontalMotor.getVelocity() < 5 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect  && Math.abs(odometry.Heading() - targetHeading) < 5){

                    autoQueued = true;

                    pathing = false;

                    collection.queueCommand(collection.extendoTargetPoint(new Point(247, 354)));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(collection.transferDrop);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140){
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("dropBasket"));

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
                    delivery.queueCommand(delivery.deposit);
                }

                if (follow.isFinished(6,6) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.deposit);
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
            telemetry.update();

            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }else{
//            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0, follow.getTurnPower(targetHeading, odometry.Heading()))));
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0, 0)));
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
            }

            if (odometry.X() < 300 && !pullDownSlides){
                pullDownSlides = true;
                delivery.queueCommand(delivery.cameraScan);
                collection.queueCommand(collection.collect);
            }

            if (follow.isFinished(6,6) && !busyDetecting){

                autoQueued = false;
                pathing = false;
                delivery.mainPivot.setPosition(delivery.findCameraScanPosition(true));

                collection.sampleSorterContour.setScanning(true);
                collection.portal.resumeStreaming();

                busyDetecting = true;
                detectionTimer.reset();
                counter = 0;
            }

            if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

                counter++;

                if (!collection.sampleSorterContour.detections.isEmpty() && counter > 5){

                    collection.sampleSorterContour.setScanning(false);
                    collection.portal.stopStreaming();
                    collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));

                    collection.queueCommand(collection.autoCollectGlobal);
                    collection.setChamberCollect(false);

                    delivery.overrideCurrent(true, delivery.stow);
                    delivery.runReset();
                    collect = true;

                    counter = 40;
                }

            }

            if (follow.isFinished(10,10) && collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                collection.queueCommand(collection.transfer);

                collection.queueCommand(delivery.transfer);

                collection.queueCommand(collection.transferDrop);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                autoQueued = true;
            }

            if (follow.isFinished(10,10) && collection.getSlidePositionCM() < 15 && collection.getClawsState() == Collection.clawState.grab && delivery.getSlidePositionCM() < 5 && collect && autoQueued){
                CycleState = cycleState.basketDrob;
                cycleBuilt = building.notBuilt;
            }

        } else if (CycleState == cycleState.basketDrob){

            if (cycleBuilt == building.notBuilt){

                follow.setPath(paths.returnPath("dropBasket"));

                targetHeading = 205;

                follow.usePathHeadings(false);

                cycleBuilt = building.built;

                pathing = true;
                drop = true;
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && odometry.X() > 280 && drop){
                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;
            }

            if (delivery.slideMotor.getCurrentPosition() > 695 && delivery.fourbarState == Delivery.fourBarState.transfer && drop){
                delivery.queueCommand(delivery.deposit);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(9,9)) {

                delivery.queueCommand(delivery.deposit);

                vertical = 1;

                pathing = false;

                drop = false;

                dropTimerDriving.reset();

            }

            if (state == targetState){

                if (!pathing && !drop && dropTimerDriving.milliseconds() > 500 && dropTimerDriving.milliseconds() < 600){
                    delivery.queueCommand(delivery.deposit);
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