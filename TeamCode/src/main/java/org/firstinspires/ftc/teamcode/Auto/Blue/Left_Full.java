package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
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

    public autoState targetState = autoState.two;
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
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(309, 303), new Vector2D(292, 308)),
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(323, 311), new Vector2D(292, 326)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(330, 330)),
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

        FtcDashboard.getInstance().startCameraStream(collection.sampleDetector, 30);

        collection.sampleDetector.closestFirst = true;

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
                    state = autoState.spikeOne;
                    built = building.notBuilt;
                }

            }

        } else if (state == autoState.spikeOne) {

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

                    targetHeading = 180;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                }

                if (odometry.X() < 320 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.deposit);
                }

                if (!autoQueued){
                    collection.targetPointWithExtendoNoArm(new Vector2D(252,306));
                }

                if (follow.isFinished(4,4) && collection.horizontalMotor.getVelocity() < 10 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect){

                    collection.setSlideTarget(collection.getSlidePositionCM());

                    pathing = false;

                    counter = 0;

                    autoQueued = true;

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(delivery.transfer);

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
                    delivery.slideSetPoint(delivery.highBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer){
                    delivery.queueCommand(delivery.deposit);
                }

                if (follow.isFinished(4,4) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.deposit);
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

                    follow.setPath(paths.returnPath("spikeTwo"));

                    targetHeading = 180;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                }

                if (odometry.X() < 320 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.deposit);
                }

                if (!autoQueued){
                    collection.targetPointWithExtendoNoArm(new Vector2D(252,331));
                }

                if (follow.isFinished(4,4) && collection.horizontalMotor.getVelocity() < 10 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect){

                    collection.setSlideTarget(collection.getSlidePositionCM());

                    pathing = false;

                    counter = 0;

                    autoQueued = true;

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(delivery.transfer);

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
                    delivery.slideSetPoint(delivery.highBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer){
                    delivery.queueCommand(delivery.deposit);
                }

                if (follow.isFinished(4,4) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.deposit);
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

                    targetHeading = 145;

                    cycleBuilt = building.built;

                    collection.queueCommand(collection.collect);

                    pathing = true;
                    pullDownSlides = false;
                    autoQueued = false;
                }

                if (odometry.X() < 320 && !pullDownSlides){
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.deposit);
                }

                if (!autoQueued){
                    collection.targetPointWithExtendoNoArm(new Vector2D(252,357));
                }

                if (follow.isFinished(4,4) && collection.horizontalMotor.getVelocity() < 10 && collection.getSlidePositionCM() > 5 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect){

                    collection.setSlideTarget(collection.getSlidePositionCM());

                    pathing = false;

                    counter = 0;

                    autoQueued = true;

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transfer);

                    collection.queueCommand(delivery.transfer);

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
                    delivery.slideSetPoint(delivery.highBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.slideMotor.getCurrentPosition() > 300 && delivery.fourbarState == Delivery.fourBarState.transfer){
                    delivery.queueCommand(delivery.deposit);
                }

                if (follow.isFinished(4,4) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                    delivery.queueCommand(delivery.deposit);
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
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(vertical,0, follow.getTurnPower(targetHeading, odometry.Heading()))));
        }
    }

}

