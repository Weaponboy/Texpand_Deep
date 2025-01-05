package org.firstinspires.ftc.teamcode.Auto.Red;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;

@Autonomous(name = "Red Right", group = "Red Autos")
public class Red_Right_Full_Auto extends OpModeEX {

    double targetHeading;

    pathsManager paths = new pathsManager();
    follower follow = new follower();

    private final sectionBuilder[] preloadDelivery = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(252, 185))
    };

    private final sectionBuilder[] obs_collecting = {
            () -> paths.addPoints(new Vector2D(250, 180), new Vector2D(280, 190), new Vector2D(332,122))
    };

    private final sectionBuilder[] clip_And_Collect_One = {
            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 182))
    };

    private final sectionBuilder[] clip_And_Collect_Two = {
            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 174))
    };

    private final sectionBuilder[] clip_And_Collect_Three = {
            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 167))
    };

    private final sectionBuilder[] spikeMarks = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(290,172),new Vector2D(290, 106))
    };

    private final sectionBuilder[] spikeMarks2 = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(310,172),new Vector2D(290, 95))
    };

    private final sectionBuilder[] spikeMarks3 = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(310,172),new Vector2D(286, 64))
    };

    private final sectionBuilder[] goToGrabPreload = {
            () -> paths.addPoints(new Vector2D(290, 68), new Vector2D(290,96),new Vector2D(294, 126))
    };

    private final sectionBuilder[] goToGrab = {
            () -> paths.addPoints(new Vector2D(260, 165), new Vector2D(280,156),new Vector2D(295, 125))
    };

    private final sectionBuilder[] goToDrop = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145),new Vector2D(260, 160))
    };

    public enum autoState {
        preLoad,
        spike_one,
        spike_two,
        spike_three,
        cycle_one,
        cycle_two,
        cycle_three,
        cycle_four,
        cycle_five,
        cycle_six,
        final_Clip,
        finished;

        public static autoState next(autoState current) {
            autoState[] values = autoState.values();
            int nextIndex = (current.ordinal() + 1) % values.length; // Wrap around to the first enum
            return values[nextIndex];
        }
    }
    public enum targetExtendo{
        notSet,
        one,
        two,
        three,
    }

    public enum CycleState{
        clip_and_collect,
        obs_collect
    }

    public enum building {
        built,
        notBuilt
    }

    boolean following = false;
    boolean collectSample = false;
    boolean clipped = false;

    autoState state = autoState.preLoad;
    autoState targetState = autoState.cycle_four;
    building built = building.notBuilt;
    building cycleBuilt = building.notBuilt;
    CycleState cycleState = CycleState.clip_and_collect;
    targetExtendo TargetExtendo = targetExtendo.notSet;

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();

    ElapsedTime extendoWait = new ElapsedTime();

    boolean runCollect = false;

    boolean firstSpike = false;

    @Override
    public void initEX() {

        delivery.setGripperState(Delivery.gripper.grab);

        odometry.startPosition(342.5, 164, 180);

        paths.addNewPath("preloadPath");
        paths.buildPath(preloadDelivery);

        paths.addNewPath("obs_collecting");
        paths.buildPath(obs_collecting);

        paths.addNewPath("clip_One");
        paths.buildPath(clip_And_Collect_One);

        paths.addNewPath("clip_Two");
        paths.buildPath(clip_And_Collect_Two);

        paths.addNewPath("clip_Three");
        paths.buildPath(clip_And_Collect_Three);

        paths.addNewPath("spike");
        paths.buildPath(spikeMarks);

        paths.addNewPath("spike2");
        paths.buildPath(spikeMarks2);

        paths.addNewPath("spike3");
        paths.buildPath(spikeMarks3);

        paths.addNewPath("goToGrabPreload");
        paths.buildPath(goToGrabPreload);

        paths.addNewPath("goToGrap");
        paths.buildPath(goToGrab);

        paths.addNewPath("goToDrob");
        paths.buildPath(goToDrop);

        follow.setPath(paths.returnPath("preloadPath"));

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        collection.sampleSorterContour.setScanning(false);
        collection.sampleSorterContour.setTargetColor(findAngleUsingContour.TargetColor.yellow);
        collection.sampleSorterContour.closestFirst = true;
    }

    @Override
    public void loopEX() {

        if (state == autoState.preLoad) {

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 180;
                following = true;
                clipped = false;
                runCollect = false;
                built = building.built;

                delivery.queueCommand(delivery.preClipFront);
            }

            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
                follow.finishPath();
                following = false;
            }

            if (targetState == autoState.final_Clip){

//                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClipFront && !collection.getChamberCollect() && !clipped){
//                    delivery.queueCommand(delivery.Clip);
//                    clipped = true;
//                }
//
//                if (follow.isFinished() && delivery.getSlidePositionCM() < 5 && collection.getSlidePositionCM() < 2) {
//                    state = autoState.final_Clip;
////                    built = building.notBuilt;
//                    runCollect = false;
//                    cycleBuilt = building.notBuilt;
//                }

            }else{

                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClipFront && !runCollect){
                    delivery.queueCommand(delivery.preClipFront);
                    delivery.queueCommand(delivery.clipFront);

                    runCollect = true;
                }


//                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClipFront && !collection.getChamberCollect() && !runCollect){
//
//                    runCollect = true;
//                    collection.sampleSorterContour.setScanning(true);
//                    collection.portal.resumeStreaming();
//
//                    delivery.mainPivot.setPosition(delivery.findCameraScanPosition(true));
//
//                    boolean detecting = true;
//                    int counter = 0;
//
//                    while (detecting && counter < 20){
//                        counter++;
//
//                        if (!collection.sampleSorterContour.detections.isEmpty() && counter > 10){
//
//                            collection.sampleSorterContour.setScanning(false);
//                            collection.portal.stopStreaming();
//                            collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));
//
//                            detecting = false;
//
//                            collection.queueCommand(collection.autoCollectChamber);
//                            collection.queueCommand(collection.chamberCollect);
//                            delivery.queueCommand(delivery.preClipFront);
//                            delivery.queueCommand(delivery.Clip);
//
//
//                        }else {
//
//                            try {
//                                Thread.sleep(50);
//                            } catch (InterruptedException e) {
//                                throw new RuntimeException(e);
//                            }
//
//                        }
//
//                    }
//
//                }else if (delivery.getSlidePositionCM() > 15 && !runCollect){
//                    delivery.mainPivot.setPosition(delivery.findCameraScanPosition(true));
//                }

                if (follow.isFinished() && delivery.getSlidePositionCM() < 5 && collection.getSlidePositionCM() < 5.5 && delivery.getCurrentCommand() != delivery.preClipFront) {
                    state = autoState.spike_one;
                    built = building.notBuilt;
                    runCollect = false;
//                    cycleBuilt = building.notBuilt;
                }
            }

        }else if (state == autoState.spike_one){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike"));
                targetHeading = 225;
                following = true;
                built = building.built;

                TargetExtendo = targetExtendo.notSet;
            }

            if (odometry.Heading() > 200 && TargetExtendo == targetExtendo.notSet){
                collection.targetPointWithExtendoNoArm(new Vector2D(248,59));
            }

            if (odometry.Heading() > 200 && TargetExtendo == targetExtendo.notSet && !firstSpike){
                collection.queueCommand(collection.preCollectNoRotate(45));
                firstSpike = true;
            }

            if (follow.isFinished(4,4) && TargetExtendo == targetExtendo.notSet){

                following = false;

                collection.queueCommand(collection.extendoTargetPoint(new Point(248,59)));
                TargetExtendo = targetExtendo.one;

            }

            if (follow.isFinished(4,4) && collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.one && collection.getSlidePositionCM() > 15){

                TargetExtendo = targetExtendo.two;

                collection.queueCommand(collection.collect);

            }else if (collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.two && targetHeading != 300){

                collection.gripServo.setPosition(35);

                collection.setClawsState(Collection.clawState.grab);

                try {
                    Thread.sleep(300);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                collection.fourBarMainPivot.setPosition(100);
                collection.fourBarSecondPivot.setPosition(280);

                follow.setExtendoHeading(true);

                targetHeading = 300;

                following = true;
            }

            if (odometry.Heading() > 280 && collection.getClawsState() == Collection.clawState.grab){
                collection.setClawsState(Collection.clawState.drop);
                state = autoState.spike_two;
                built = building.notBuilt;
                follow.setExtendoHeading(false);
            }

        } else if (state == autoState.spike_two){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike2"));
                targetHeading = 235;
                following = true;
                built = building.built;
                TargetExtendo = targetExtendo.notSet;
            }

            if (follow.isFinished(4,4) && TargetExtendo == targetExtendo.notSet && Math.abs(odometry.Heading() - targetHeading) < 5){
                collection.queueCommand(collection.preCollectNoRotate(45));
//                collection.queueCommand(collection.collect);
//                collection.griperRotate.setPosition(50);
                collection.queueCommand(collection.extendoTargetPoint(new Point(246,33.5)));
                TargetExtendo = targetExtendo.one;
            }

            if (follow.isFinished(4,4) && collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.one && collection.getSlidePositionCM() > 15){

                TargetExtendo = targetExtendo.two;

                collection.queueCommand(collection.collect);

            }else if (collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.two && targetHeading != 292){

                collection.gripServo.setPosition(35);

                collection.setClawsState(Collection.clawState.grab);

                try {
                    Thread.sleep(300);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                collection.fourBarMainPivot.setPosition(100);
                collection.fourBarSecondPivot.setPosition(280);

                follow.setExtendoHeading(true);

                targetHeading = 292;
            }

            if (odometry.Heading() > 272 && collection.getClawsState() == Collection.clawState.grab){
                collection.setClawsState(Collection.clawState.drop);
                state = autoState.spike_three;
                built = building.notBuilt;
                follow.setExtendoHeading(false);
                collection.setSlideTarget(collection.getSlideTarget()-15);
            }

        } else if (state == autoState.spike_three){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike3"));
                targetHeading = 235;
                following = true;
                built = building.built;
                TargetExtendo = targetExtendo.notSet;
            }

            if (follow.isFinished(4,4) && TargetExtendo == targetExtendo.notSet && Math.abs(odometry.Heading() - targetHeading) < 3){
                collection.queueCommand(collection.preCollectNoRotate(45));
//                collection.queueCommand(collection.collect);
//                collection.griperRotate.setPosition(50);
                collection.queueCommand(collection.extendoTargetPoint(new Point(247,6.5)));
                TargetExtendo = targetExtendo.one;
            }

            if (follow.isFinished(4,4) && collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.one && collection.getSlidePositionCM() > 15){

                TargetExtendo = targetExtendo.two;

                collection.queueCommand(collection.collect);

            }else if (collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.two && targetHeading != 292){

                collection.gripServo.setPosition(35);

                collection.setClawsState(Collection.clawState.grab);

                try {
                    Thread.sleep(300);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                collection.fourBarMainPivot.setPosition(100);
                collection.fourBarSecondPivot.setPosition(280);

                collection.setSlideTarget(collection.getSlideTarget()-15);

                follow.setExtendoHeading(true);

                targetHeading = 292;
            }

            if (odometry.Heading() > 272 && collection.getClawsState() == Collection.clawState.grab){
                collection.setClawsState(Collection.clawState.drop);
                state = autoState.cycle_one;
                built = building.notBuilt;
                runCollect = false;
                cycleBuilt = building.notBuilt;
                follow.setExtendoHeading(true);
            }

        }else {

            if (built == building.notBuilt) {

                cycleBuilt = building.notBuilt;

                cycleState = CycleState.obs_collect;

                following = true;

                built = building.built;

            }

            fullCycle();

        }

        if (state == autoState.finished) {
            requestOpModeStop();
        }

        odometry.queueCommand(odometry.updateLineBased);

        if (following) {
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
        }else {
//            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0, 0,follow.getTurnPower(targetHeading, odometry.Heading()))));
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0, 0,0)));
        }
    }

    public void fullCycle(){

        Point targetExtendoPoint = new Point(333.5, 70.5);

        if (cycleState == CycleState.obs_collect) {

            if (cycleBuilt == building.notBuilt) {
                cycleBuilt = building.built;

                following = true;
                collectSample = false;

                if (!(state == autoState.cycle_one)){
                    targetHeading = 307;
                    follow.setPath(paths.returnPath("goToGrap"));
                }else{

                }

                follow.setPath(paths.returnPath("goToGrabPreload"));

                collection.queueCommand(collection.preCollectNoRotate(180));

                delivery.griperRotateSev.setPosition(0);
            }

            if (odometry.X() > 270){
                targetHeading = 307;
            }

            if (odometry.Heading() > 290 && odometry.Heading() < 312 && !collectSample && state != autoState.cycle_one){
                collection.targetPointWithExtendo(new Vector2D(333.5, 70.5));
                delivery.griperRotateSev.setPosition(0);
            }

            if (follow.isFinished(4,4) && odometry.getYVelocity() < 10 && odometry.getXVelocity() < 10 && Math.abs(odometry.Heading() - targetHeading) < 5 && !collectSample){

                following = false;

                delivery.griperRotateSev.setPosition(0);

                collection.queueCommand(collection.extendoTargetPoint(targetExtendoPoint));

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transfer);

                collection.queueCommand(delivery.transfer);

                collection.queueCommand(collection.transferDrop);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

                collectSample = true;

            } else if (follow.isFinished(4,4) && collectSample && collection.getSlideTarget() == 0){
                cycleState = CycleState.clip_and_collect;
                cycleBuilt = building.notBuilt;
            }
//
//            if (!following && collection.getFourBarState() == Collection.fourBar.preCollect && collection.getCurrentCommand() == collection.defaultCommand && collectSample){
//
//                collection.queueCommand(collection.extendoTargetPoint(new Point(325, 84)));
//
//                collection.queueCommand(collection.extendoTargetPoint(targetExtendoPoint));
//
//                collection.queueCommand(collection.collect);
//
//                collection.queueCommand(collection.transfer);
//
//                collection.queueCommand(delivery.transfer);
//
//                collection.queueCommand(collection.transferDrop);
//
//                collection.queueCommand(delivery.closeGripper);
//
//                collection.queueCommand(collection.openGripper);
//
//            }

        }else if (cycleState == CycleState.clip_and_collect) {

            if (cycleBuilt == building.notBuilt) {

                cycleBuilt = building.built;
                follow.setPath(paths.returnPath("goToDrob"));
                following = true;
                clipped = false;

                targetHeading = 337;
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && collection.slidesReset.isPressed() && !clipped && delivery.fourbarState == Delivery.fourBarState.transfer){
                delivery.griperRotateSev.setPosition(90);
                delivery.queueCommand(delivery.preClipBack);
            }

            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
                following = false;
                follow.finishPath();
            }

            if (follow.isFinished(2,2) && delivery.fourbarState == Delivery.fourBarState.preClip){
                delivery.queueCommand(delivery.clipBack);
                clipped = true;
            }

            if (state == targetState){
                if (follow.isFinished(2, 2) && delivery.getSlidePositionCM() < 5 && clipped && delivery.getCurrentCommand() != delivery.preClipFront && delivery.fourbarState == Delivery.fourBarState.transfer) {
                    state = autoState.finished;
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                }
            }else{
                if (follow.isFinished(2, 2) && clipped && delivery.getCurrentCommand() != delivery.preClipFront && delivery.fourbarState == Delivery.fourBarState.clip) {
                    state = autoState.next(state);
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                    collection.setSlideTarget(45);
                    follow.setExtendoHeading(false);
                }
            }

        }

    }

    public pathBuilder getClipPath(){

        if (state == autoState.cycle_one){
            return paths.returnPath("clip_One");
        }else if (state == autoState.cycle_two){
            return paths.returnPath("clip_Two");
        }else if (state == autoState.cycle_three){
            return paths.returnPath("clip_Three");
        }else {
            return paths.returnPath("preloadPath");
        }

    }
}
