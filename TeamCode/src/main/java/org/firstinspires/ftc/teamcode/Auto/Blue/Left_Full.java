package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Red.Red_Right_Full_Auto;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
@Autonomous
public class Left_Full extends OpModeEX {
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

    public autoState targetState = autoState.four;
    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public building cycleBuilt = building.notBuilt;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(334, 334)),
    };

    private final sectionBuilder[] subCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(204, 288), new Vector2D(200, 238)),
    };

    private final sectionBuilder[] spikeOne = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(309, 303), new Vector2D(292, 306)),
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(323, 311), new Vector2D(292, 326)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(334, 330)),
    };




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

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        collection.sampleSorterContour.closestFirst = true;

        follow.setPath(paths.returnPath("collectSub"));

        paths.addNewPath("spikeOne");

        paths.buildPath(spikeOne);

        follow.setPath(paths.returnPath("spikeOne"));

        paths.addNewPath("spikeTwo");

        paths.buildPath(spikeTwo);

        follow.setPath(paths.returnPath("spikeTwo"));

    }

    @Override
    public void loopEX() {

        if (state == autoState.preload) {

            if (built == building.notBuilt) {

                delivery.slideSetPoint(delivery.highBasket);
                delivery.slides = Delivery.slideState.moving;

                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 235;
                built = building.built;
                drop = true;
                dropTimer.reset();

                pathing = true;
            }

            if (delivery.slideMotor.getCurrentPosition() > 250 && delivery.fourbarState == Delivery.fourBarState.transfer){
                delivery.queueCommand(delivery.deposit);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > delivery.highBasket - 4 && follow.isFinished(4,4)) {
                delivery.queueCommand(delivery.deposit);

                pathing = false;

                drop = false;
            }

            if (follow.isFinished(4, 4) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop) {
                if (state == targetState){
                    state = autoState.finished;
                }else{
                    state = autoState.one;
                    built = building.notBuilt;
                }

            }

        } else if (state == autoState.spikeOne) {

            if (built == building.notBuilt) {

                built = building.built;
                drop = true;
                dropTimer.reset();

                pathing = true;
            }


            if (CycleState == cycleState.spikeCollect){

                if (cycleBuilt == building.notBuilt){
                    follow.setPath(paths.returnPath("spikeOne"));
                    targetHeading = 180;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);
                }
                collection.targetPointWithExtendo(new Vector2D(246,302.5));
                if (collection.horizontalMotor.getVelocity()<10){
                    collection.queueCommand(collection.collect);
                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(delivery.transfer);

                    collection.queueCommand(collection.transferDrop);

                    collection.queueCommand(delivery.closeGripper);

                    collection.queueCommand(collection.openGripper);

                }





            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt){

                    follow.setPath(paths.returnPath("dropBasket"));

                    targetHeading = 205;

                    cycleBuilt = building.built;
                    pathing = true;
                    drop = true;
                }

            }


        }




        if (state==autoState.finished){
            requestOpModeStop();
        }

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
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0, follow.getTurnPower(targetHeading, odometry.Heading()))));
        }
    }

    public void subCycle () {

        if (CycleState == cycleState.subCollect){

            if (cycleBuilt == building.notBuilt){
                follow.setPath(paths.returnPath("collectSub"));
                targetHeading = 225;

                cycleBuilt = building.built;

                busyDetecting = false;
                pathing = true;
                pullDownSlides = false;
                collect = false;
            }

            if (odometry.X() < 240){
                targetHeading = 270;
            }

            if (odometry.X() < 300 && !pullDownSlides){
                pullDownSlides = true;
                delivery.queueCommand(delivery.cameraScan);
            }

            if (follow.isFinished(4,4) && !busyDetecting){

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

                if (!collection.sampleSorterContour.detections.isEmpty() && counter > 8){
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

            if (follow.isFinished(4,4) && collection.getCurrentCommand() != collection.autoCollectGlobal && collect && !autoQueued){

                collection.queueCommand(collection.transfer);

                collection.queueCommand(delivery.transfer);

                collection.queueCommand(collection.transferDrop);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                autoQueued = true;
            }

            if (follow.isFinished(4,4) && collection.getSlidePositionCM() < 15 && collection.getCurrentCommand() != collection.autoCollectGlobal && delivery.getSlidePositionCM() < 5 && collect && autoQueued){
                CycleState = cycleState.basketDrob;
                cycleBuilt = building.notBuilt;
            }

        } else if (CycleState == cycleState.basketDrob){

            if (cycleBuilt == building.notBuilt){

                follow.setPath(paths.returnPath("dropBasket"));

                targetHeading = 205;

                cycleBuilt = building.built;

                if (!(state == autoState.two)){

                }

                pathing = true;
                drop = true;
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && odometry.X() > 280 && drop){
                delivery.slideSetPoint(delivery.highBasket);
                delivery.slides = Delivery.slideState.moving;
            }

            if (delivery.slideMotor.getCurrentPosition() > 200 && delivery.fourbarState == Delivery.fourBarState.transfer && drop){
                delivery.queueCommand(delivery.deposit);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > delivery.highBasket - 4 && follow.isFinished(9,9)) {

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
                }

            }

        }

    }
}

