package org.firstinspires.ftc.teamcode.Auto.Specimen;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "Spec_Full", group = "AA Comp Autos")
public class Spec_Full extends OpModeEX {

    double targetHeading;

    pathsManager paths = new pathsManager();
    follower follow = new follower();

    double adjustedTarget = 0;
    Vector2D powerPID = new Vector2D();

    boolean retryCollection = false;
    ElapsedTime retryTimer = new ElapsedTime();

    private final sectionBuilder[] preloadDelivery = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(255, 192))
    };

    private final sectionBuilder[] obs_collecting = {
            () -> paths.addPoints(new Vector2D(250, 180), new Vector2D(280, 190), new Vector2D(332,122))
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

    private final sectionBuilder[] goToDrop1 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145),new Vector2D(257, 185))
    };

    private final sectionBuilder[] goToDrop2 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145),new Vector2D(257, 175))
    };

    private final sectionBuilder[] goToDrop3 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145),new Vector2D(257, 165))
    };

    private final sectionBuilder[] goToDrop4 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145),new Vector2D(257, 155))
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
    boolean ranPreClip = false;
    boolean headingAdjustment = false;
    boolean headingOverride = true;

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

    boolean PIDToPoint = false;
    boolean firstSpike = false;

    @Override
    public void initEX() {

        delivery.setGripperState(Delivery.gripper.grab);

        odometry.startPosition(342.5, 164, 180);

        paths.addNewPath("preloadPath");
        paths.buildPath(preloadDelivery);

        paths.addNewPath("spike");
        paths.buildPath(spikeMarks);

        paths.addNewPath("spike2");
        paths.buildPath(spikeMarks2);

        paths.addNewPath("spike3");
        paths.buildPath(spikeMarks3);

        paths.addNewPath("goToDrob_cycle_one");
        paths.buildPath(goToDrop1);

        paths.addNewPath("goToDrob_cycle_two");
        paths.buildPath(goToDrop2);

        paths.addNewPath("goToDrob_cycle_three");
        paths.buildPath(goToDrop3);

        paths.addNewPath("goToDrob_cycle_four");
        paths.buildPath(goToDrop4);

        follow.setPath(paths.returnPath("preloadPath"));

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
                PIDToPoint = false;
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
                collection.angle = -45;
                collection.targetPointWithExtendoNoArm(new Vector2D(246,57.5));
            }

            if (odometry.Heading() > 200 && TargetExtendo == targetExtendo.notSet && !firstSpike){
                collection.queueCommand(collection.preCollectNoRotate(45));
                firstSpike = true;
            }

            if (follow.isFinished(4,4) && TargetExtendo == targetExtendo.notSet){

                following = false;

                collection.angle = 45;

                collection.queueCommand(collection.extendoTargetPoint(new Vector2D(243,55)));
                TargetExtendo = targetExtendo.one;

            }

            if (follow.isFinished(4,4) && collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.one && collection.getSlidePositionCM() > 15){

                TargetExtendo = targetExtendo.two;

                collection.queueCommand(collection.collect);

            }else if (collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.two && targetHeading != 280){

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

                targetHeading = 280;

                following = true;
            }

            if (odometry.Heading() > 270 && collection.getClawsState() == Collection.clawState.grab){
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

                collection.angle = 45;
                collection.queueCommand(collection.extendoTargetPoint(new Vector2D(242,29)));
                TargetExtendo = targetExtendo.one;
            }

            if (follow.isFinished(4,4) && collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.one && collection.getSlidePositionCM() > 15){

                TargetExtendo = targetExtendo.two;

                collection.queueCommand(collection.collect);

            }else if (collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.two && targetHeading != 280){

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

                targetHeading = 280;
            }

            if (odometry.Heading() > 268 && collection.getClawsState() == Collection.clawState.grab){
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
                collection.angle = 45;
                collection.queueCommand(collection.extendoTargetPoint(new Vector2D(240,4)));
                TargetExtendo = targetExtendo.one;
            }

            if (follow.isFinished(4,4) && collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.one && collection.getSlidePositionCM() > 15){

                TargetExtendo = targetExtendo.two;

                collection.queueCommand(collection.collect);

            }else if (collection.getCurrentCommand() == collection.defaultCommand && TargetExtendo == targetExtendo.two && targetHeading != 285){

                collection.gripServo.setPosition(35);

                collection.setClawsState(Collection.clawState.grab);

                try {
                    Thread.sleep(300);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                collection.fourBarMainPivot.setPosition(100);
                collection.fourBarSecondPivot.setPosition(280);

                collection.setSlideTarget(collection.getSlideTarget()-30);

                follow.setExtendoHeading(true);

                targetHeading = 285;
            }

            if (odometry.Heading() > 270 && collection.getClawsState() == Collection.clawState.grab){
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

//                System.out.println("built state: " + state.name());

            }

            fullCycle();

        }

        if (state == autoState.finished) {
            requestOpModeStop();
        }

        odometry.queueCommand(odometry.updateLineBased);

        if (following) {
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());

//            telemetry.addData("Loop time", loopTime);
//            telemetry.addData("Y", odometry.Y());
//            telemetry.addData("Heading", odometry.Heading());
//            telemetry.addData("X", odometry.X());
//            telemetry.addData("getVertical", currentPower.getVertical());
//            telemetry.addData("getHorizontal", currentPower.getHorizontal());
//            telemetry.addData("getPivot", currentPower.getPivot());
//            telemetry.update();

//            System.out.println("Running pathing code");
//
//            System.out.println("Running X" + currentPower.getVertical());
//            System.out.println("Running Y" + currentPower.getHorizontal());
//
//            System.out.println("Error: " + follow.getErrorToEnd());
//
//            System.out.println("X" + odometry.X());
//            System.out.println("Y" + odometry.Y());

            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }else {
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

//        telemetry.addData("Y", odometry.Y());
//        telemetry.addData("Heading", odometry.Heading());
//        telemetry.addData("X", odometry.X());
//        telemetry.addData("Current command default? ", collection.getCurrentCommand() == collection.defaultCommand);
//        telemetry.addData("pathing ", following);
//        telemetry.addData("boolean second", Math.abs(targetHeading - odometry.Heading()));
//        telemetry.addData("", "");
//        telemetry.addData("target Point", limelight.getTargetPoint());
//        telemetry.update();
    }

    public void fullCycle(){

        Vector2D targetExtendoPoint = new Vector2D(344, 82);

        if (cycleState == CycleState.obs_collect) {

            if (PIDToPoint) {
                PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(290, 141), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
            } else {
                powerPID = new Vector2D();
            }

            if (cycleBuilt == building.notBuilt) {
                cycleBuilt = building.built;

                following = false;
                collectSample = false;
                headingOverride = false;
                PIDToPoint = true;
                targetHeading = 319;
                retryCollection = false;

                collection.resetTransferCanceled();
                collection.setCancelTransfer(true);

                collection.queueCommand(collection.preCollectNoRotate(180));

                delivery.griperRotateSev.setPosition(0);
            }

//            if (odometry.X() > 270){
//                targetHeading = 307;
//            }

            if (odometry.Heading() > 310 && odometry.Heading() < 325 && !collectSample && state != autoState.cycle_one && !retryCollection){
                collection.targetPointWithExtendo(targetExtendoPoint);
                delivery.griperRotateSev.setPosition(0);
            }

            if (!retryCollection && Math.abs(odometry.getYVelocity()) < 5 && Math.abs(odometry.getXVelocity()) < 5 && Math.abs(odometry.Heading() - targetHeading) < 5 && !collectSample){

                following = false;

                delivery.griperRotateSev.setPosition(0);

                collection.angle = 90;

                collection.queueCommand(collection.extendoTargetPoint(targetExtendoPoint));

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transfer(Collection.tranfer.specimen));

                collectSample = true;

            } else if (collectSample && collection.getSlideTarget() == 0 && Math.abs(collection.horizontalMotor.getVelocity()) > 20){
                cycleState = CycleState.clip_and_collect;
                cycleBuilt = building.notBuilt;
            } else if (collectSample && retryCollection && collection.isTransferCanceled()){
                cycleState = CycleState.clip_and_collect;

                cycleBuilt = building.notBuilt;

                collection.setCancelTransfer(false);

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transfer(Collection.tranfer.specimen));
            }

            if (retryCollection && collection.getCurrentCommand() == collection.returnDefaultCommand() && retryTimer.milliseconds() > 1000 && !collectSample){
                collection.queueCommand(collection.extendoTargetPoint(targetExtendoPoint));

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transfer(Collection.tranfer.specimen));

                collectSample = true;
            }

            if (collection.isTransferCanceled() && collectSample && !retryCollection){
                retryCollection = true;

                collectSample = false;

                retryTimer.reset();

                collection.resetTransferCanceled();

                collection.setSlideTarget(30);
            }

        }else if (cycleState == CycleState.clip_and_collect) {

            if (cycleBuilt == building.notBuilt) {

                cycleBuilt = building.built;

                follow.setPath(paths.returnPath("goToDrob_" + state.name()));

                following = true;
                PIDToPoint = false;
                clipped = false;
                ranPreClip = false;

                targetHeading = 345;
            }

            if (!ranPreClip && collection.getCurrentCommand() == collection.defaultCommand && collection.slidesReset.isPressed() && !clipped && delivery.fourbarState == Delivery.fourBarState.transfer){
                delivery.griperRotateSev.setPosition(90);
                delivery.queueCommand(delivery.preClipBack);
                ranPreClip = true;
            }

            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && odometry.X() < 270){
                following = false;
                follow.finishPath();
            }

            if (follow.isFinished() && delivery.fourbarState == Delivery.fourBarState.preClip){
                delivery.queueCommand(delivery.clipBack);
                delivery.queueCommand(delivery.releaseClip);
                clipped = true;
            }

            if (state == targetState){
                if (follow.isFinished() && delivery.getSlidePositionCM() < 5 && clipped && delivery.getCurrentCommand() != delivery.preClipFront && delivery.fourbarState == Delivery.fourBarState.transfer) {
                    state = autoState.finished;
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
//                    System.out.println("finished: " + state.name());
                }
            }else{
                if (follow.isFinished() && clipped && delivery.getCurrentCommand() != delivery.preClipFront && delivery.fourbarState == Delivery.fourBarState.clip) {
                    state = autoState.next(state);
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                    collection.setSlideTarget(30);
                    follow.setExtendoHeading(false);
//                    System.out.println("incremented state: " + state.name());
                }
            }

        }

    }
}
