package org.firstinspires.ftc.teamcode.Auto.Specimen;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.command_library.Subsystems.Limelight;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "Spec_Full", group = "AA Comp Autos")
public class Spec_Full extends OpModeEX {

    pathsManager paths = new pathsManager();
    follower follow = new follower();
    Vector2D powerPID = new Vector2D();
    ElapsedTime retryTimer = new ElapsedTime();

    private final sectionBuilder[] preloadDelivery = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(255, 192))
    };

    private final sectionBuilder[] deliverSubSample = {
            () -> paths.addPoints(new Vector2D(252, 192), new Vector2D(300,198), new Vector2D(309, 137))
    };

    private final sectionBuilder[] spikeMarks1 = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(290,172), new Vector2D(290, 116))
    };

    private final sectionBuilder[] spikeMarks2 = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(310,172), new Vector2D(290, 90))
    };

    private final sectionBuilder[] spikeMarks3 = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(310,172), new Vector2D(290, 65))
    };

    private final sectionBuilder[] spikeMarks3Drop = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(310,172), new Vector2D(300, 70))
    };

    private final sectionBuilder[] goToDrop1 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145), new Vector2D(258, 162))
    };

    private final sectionBuilder[] goToDrop2 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145), new Vector2D(258, 166))
    };

    private final sectionBuilder[] goToDrop3 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145), new Vector2D(258, 170))
    };

    private final sectionBuilder[] goToDrop4 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145),new Vector2D(258, 174))
    };

    private final sectionBuilder[] goToDrop5 = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145),new Vector2D(258, 178))
    };

    Vector2D collectObservationZonePoint = new Vector2D(292, 127);

    private final sectionBuilder[] collectObservationZone = {
            () -> paths.addPoints(new Vector2D(250, 180), new Vector2D(265,170), collectObservationZonePoint)
    };

    public enum autoState {
        preload,
        deliverObsSpec,
        spike_one,
        spike_two,
        spike_three,
        cycle_one,
        cycle_two,
        cycle_three,
        cycle_four,
        cycle_five,
        finished;

        public static autoState next(autoState current) {
            autoState[] values = autoState.values();
            int nextIndex = (current.ordinal() + 1) % values.length; // Wrap around to the first enum
            return values[nextIndex];
        }
    }
    public enum visionPreload {
        driving,
        detecting,
        collecting,
        retryingCollection,
        doneGotOne,
        doneFailed
    }

    public enum CycleState{
        clip,
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
    boolean busyDetecting = false;
    boolean retryCollection = false;
    boolean runCollect = false;
    boolean PIDToPoint = false;
    boolean firstSpike = false;
    boolean collect = false;

    double targetHeading;
    double adjustedTarget = 0;
    int counter = 0;

    autoState state = autoState.preload;
    autoState targetState = autoState.cycle_five;
    building built = building.notBuilt;
    building cycleBuilt = building.notBuilt;
    CycleState cycleState = CycleState.clip;
    visionPreload visionStates = visionPreload.driving;
    ElapsedTime detectionTimer = new ElapsedTime();

    Vector2D spikeOne = new Vector2D(244,55);
    Vector2D spikeTwo = new Vector2D(244.5,30);
    Vector2D spikeThree = new Vector2D(244.5,5.5);

    @Override
    public void initEX() {

        delivery.setGripperState(Delivery.gripper.grab);

        odometry.startPosition(342.5, 164, 180);

        paths.addNewPath("preloadPath");
        paths.buildPath(preloadDelivery);

        paths.addNewPath("subSample");
        paths.buildPath(deliverSubSample);

        paths.addNewPath("spike1");
        paths.buildPath(spikeMarks1);

        paths.addNewPath("spike2");
        paths.buildPath(spikeMarks2);

        paths.addNewPath("spike3");
        paths.buildPath(spikeMarks3);

        paths.addNewPath("spike3drop");
        paths.buildPath(spikeMarks3Drop);

        paths.addNewPath("goToDrob_cycle_one");
        paths.buildPath(goToDrop1);

        paths.addNewPath("goToDrob_cycle_two");
        paths.buildPath(goToDrop2);

        paths.addNewPath("goToDrob_cycle_three");
        paths.buildPath(goToDrop3);

        paths.addNewPath("goToDrob_cycle_four");
        paths.buildPath(goToDrop4);

        paths.addNewPath("goToDrob_cycle_five");
        paths.buildPath(goToDrop5);

        paths.addNewPath("collectObservationZone");
        paths.buildPath(collectObservationZone);

        follow.setPath(paths.returnPath("preloadPath"));

    }

    @Override
    public void loopEX() {

        if (state == autoState.preload) {

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 180;

                following = true;
                clipped = false;
                collect = false;

                built = building.built;

                delivery.queueCommand(delivery.preClipFront);
                collection.queueCommand(collection.visionScan);

                visionStates = visionPreload.driving;
                limelight.setTargetColor(Limelight.color.red);

                collection.targetPositionManuel = new Vector2D(6, 20);
                collection.armEndPointIncrement(14, -4, false);

                System.out.println("Built program" + odometry.X());
            }

            switch (visionStates){
                case driving:

                    if (follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && !busyDetecting && !collect){

                        limelight.setReturningData(true);
                        limelight.setGettingResults(true);

                        limelight.setTargetColor(Limelight.color.red);

                        busyDetecting = true;
                        detectionTimer.reset();
                        counter = 0;

                        collect = false;
                        following = false;
                        PIDToPoint = false;

                        visionStates = visionPreload.detecting;

                        System.out.println("Ran Targeting" + odometry.getXVelocity());
                    }

                    if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
                        follow.finishPath();
                        following = false;
                        PIDToPoint = false;

                        System.out.println("Finished path" + odometry.X());
                    }

                    break;
                case detecting:

                    if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20 && !collect){

                        counter++;

                        if (limelight.getTargetPoint() != null && counter > 2){

                            System.out.println("tareget X" + limelight.getTargetPoint().getTargetPoint().getX());
                            System.out.println("tareget Y" + limelight.getTargetPoint().getTargetPoint().getY());

                            collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                            delivery.queueCommand(delivery.clipFront);
                            clipped = true;

                            collect = true;
                            busyDetecting = false;
                            counter = 40;

                            visionStates = visionPreload.collecting;

                        }

                    }

//                    else if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter > 20) {
//
//                        delivery.queueCommand(delivery.clipFront);
//                        clipped = true;
//
//                        visionStates = visionPreload.doneFailed;
//x
//                        busyDetecting = false;
//                    }

                    break;
                case collecting:

                    if (collect && collection.getFourBarState() == Collection.fourBar.collect && collection.getCurrentCommand() == collection.returnDefaultCommand()){
                        collection.queueCommand(collection.transferNoSave(Collection.tranfer.chamberCollect));
                    }

//                    if (delivery.getSlidePositionCM() > 10 && !clipped && collect){
//                        delivery.queueCommand(delivery.clipFront);
//                        clipped = true;
//                    }

                    if (collection.getSlidePositionCM() < 5 && collection.getClawsState() == Collection.clawState.grab && clipped){
                        visionStates = visionPreload.doneGotOne;
                    }

                    break;
                case retryingCollection:
                    break;
                case doneGotOne:

                    if (delivery.getGripperState() == Delivery.gripper.drop && collection.getSlidePositionCM() < 5) {
                        state = autoState.deliverObsSpec;
                        built = building.notBuilt;
                        clipped = false;
                    }

                    break;
                case doneFailed:

                    if (follow.isFinished() && delivery.getGripperState() == Delivery.gripper.drop && collection.getSlidePositionCM() < 5) {
                        state = autoState.spike_one;
                        built = building.notBuilt;
                        clipped = false;
                    }

                    break;
                default:
            }

        } else if (state == autoState.deliverObsSpec) {

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("subSample"));
                targetHeading = 295;
                following = true;
                runCollect = false;
                built = building.built;
            }

            if (odometry.Heading() > 270 && collection.getSlideTarget() != 48){
                collection.setSlideTarget(48);
                collection.queueCommand(collection.observationDrop);
                collection.manualAngle = 90;
            }

            if (follow.isFinished(4, 4) && collection.getSlidePositionCM() > 40){
                collection.setClawsState(Collection.clawState.drop);
                collection.setSlideTarget(20);

                state = autoState.spike_one;
                built = building.notBuilt;
                clipped = false;
            }

        } else if (state == autoState.spike_one){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike1"));
                targetHeading = 235;
                following = true;
                runCollect = false;
                built = building.built;
                collection.queueCommand(collection.preCollectNoRotate(45));
            }

            if (odometry.Heading() > 225 && odometry.Heading() < 245 && odometry.Y() < 140 && !firstSpike){
                collection.angle = 45;
                collection.targetPointWithExtendo(spikeOne);
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && !following && runCollect){

                follow.setExtendoHeading(true);

                targetHeading = 295;

                following = true;

                headingOverride = false;

                follow.setPath(paths.returnPath("spike2"));

                collection.turret.setPosition(190);
            }

            if (follow.isFinished(5,5) && Math.abs(odometry.Heading() - targetHeading) < 10 && !runCollect){
                following = false;

                headingOverride = true;

                runCollect = true;

                collection.angle = 45;

                firstSpike = true;

                collection.queueCommand(collection.extendoTargetPoint(spikeOne));

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transferNoSave(Collection.tranfer.obsSpikes));
            }

            if (odometry.Heading() > 290 && collection.getClawsState() == Collection.clawState.grab){
                collection.setClawsState(Collection.clawState.drop);
                state = autoState.spike_two;
                built = building.notBuilt;
                follow.setExtendoHeading(false);
            }

        } else if (state == autoState.spike_two){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike2"));
                targetHeading = 245;
                following = true;
                runCollect = false;
                clipped = false;
                built = building.built;
                collection.queueCommand(collection.preCollectNoRotate(45));
            }

            if (odometry.Heading() > 235 && odometry.Heading() < 255 && odometry.Y() < 140 && !firstSpike){
                collection.angle = 45;
                collection.targetPointWithExtendo(spikeTwo);
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && !following && runCollect){

                follow.setExtendoHeading(true);

                targetHeading = 285;

                following = true;

                headingOverride = false;

                collection.turret.setPosition(190);
            }

            if (follow.isFinished(5,5) && Math.abs(odometry.Heading() - targetHeading) < 10 && !runCollect){
                following = false;

                headingOverride = true;

                runCollect = true;

                collection.angle = 45;

                firstSpike = true;

                collection.queueCommand(collection.extendoTargetPoint(spikeTwo));

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transferNoSave(Collection.tranfer.obsSpikes));
            }

            if (!clipped && collection.getClawsState() == Collection.clawState.grab){
                collection.setSlideTarget(collection.getSlideTarget()-20);
                clipped = true;
            }

            if (odometry.Heading() > 280 && collection.getClawsState() == Collection.clawState.grab && collection.getCurrentCommand() == collection.returnDefaultCommand()){
                collection.setClawsState(Collection.clawState.drop);
                collection.setSlideTarget(collection.getSlideTarget()-20);
                state = autoState.spike_three;
                built = building.notBuilt;
                follow.setExtendoHeading(false);
            }


        } else if (state == autoState.spike_three){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike3"));
                targetHeading = 242;
                following = true;
                runCollect = false;
                built = building.built;
                collection.queueCommand(collection.preCollectNoRotate(45));
            }

            if (odometry.Heading() > 232 && odometry.Heading() < 252 && odometry.Y() < 140 && !firstSpike){
                collection.angle = 45;
                collection.targetPointWithExtendo(spikeThree);
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && !following && runCollect){

                follow.setExtendoHeading(true);

                collection.setSlideTarget(collection.getSlideTarget()-15);

                targetHeading = 295;

                following = true;

                headingOverride = false;

                follow.setPath(paths.returnPath("spike3drop"));
            }

            if (follow.isFinished(5,5) && Math.abs(odometry.Heading() - targetHeading) < 10 && !runCollect){
                following = false;

                headingOverride = true;

                runCollect = true;

                collection.angle = 45;

                firstSpike = true;

                collection.queueCommand(collection.extendoTargetPoint(spikeThree));

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transferNoSave(Collection.tranfer.obsSpikes));
            }

            if (odometry.Heading() > 290 && collection.getClawsState() == Collection.clawState.grab && collection.getCurrentCommand() == collection.returnDefaultCommand()){
                collection.setClawsState(Collection.clawState.drop);
                state = autoState.cycle_one;
                built = building.notBuilt;
                follow.setExtendoHeading(false);
            }


        } else if (state == autoState.cycle_one) {

            if (built == building.notBuilt) {

                cycleBuilt = building.notBuilt;

                cycleState = CycleState.obs_collect;

//                following = true;

                built = building.built;

            }

            //point to target for collection
            Vector2D targetExtendoPoint = new Vector2D(328.8, 3);
            Vector2D pidTarget = new Vector2D(290, 141);

            if (cycleState == CycleState.obs_collect) {

                if (cycleBuilt == building.notBuilt) {

                    //build state set
                    cycleBuilt = building.built;

                    //reset booleans
//                    following = false;
                    collectSample = false;
                    headingOverride = false;
                    retryCollection = false;

                    //set target heading
                    targetHeading = 292;

                    //reset and enable transfer fail detection
                    collection.resetTransferCanceled();
                    collection.setCancelTransfer(true);

                    //reset and enable transfer fail detection
                    collection.queueCommand(collection.preCollectNoRotate(180));

                    //reset delivery gripper to transfer position
                    delivery.griperRotateSev.setPosition(90);

                }

                /**
                 * run collection commands
                 * */
                if (!retryCollection && Math.abs(odometry.Heading() - targetHeading) < 8 && !collectSample){

                    // Reset booleans
                    following = false;
                    collectSample = true;
                    PIDToPoint = false;

                    // Queue collection and transfer
                    collection.angle = 90;

                    collection.queueCommand(collection.extendoTargetPoint(targetExtendoPoint));
                    collection.queueCommand(collection.collect);
                    collection.queueCommand(collection.transfer(Collection.tranfer.specimen));

                    // Delivery rotate reset to transfer
                    delivery.griperRotateSev.setPosition(90);

                } else if (collectSample && collection.getSlideTarget() == 0 && Math.abs(collection.horizontalMotor.getVelocity()) > 20){

                    cycleState = CycleState.clip;
                    cycleBuilt = building.notBuilt;

                } else if (collectSample && retryCollection && collection.isTransferCanceled()){

                    cycleState = CycleState.clip;
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

                    collection.stopTargeting();

                    collection.setSlideTarget(20);

                }

            }else if (cycleState == CycleState.clip) {

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
                    delivery.griperRotateSev.setPosition(10);
                    delivery.queueCommand(delivery.preClipBackAuto);
                    ranPreClip = true;
                }

//                if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && odometry.X() < 270){
//                    following = false;
//                    follow.finishPath();
//                }

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
                        System.out.println("finished: " + state.name());
                    }
                }else{
                    if (follow.isFinished() && clipped && delivery.getCurrentCommand() != delivery.preClipFront && delivery.fourbarState == Delivery.fourBarState.clip) {
                        state = autoState.next(state);
                        built = building.notBuilt;
                        cycleBuilt = building.notBuilt;
                        collection.setSlideTarget(30);
                        follow.setExtendoHeading(false);
                        System.out.println("incremented state: " + state.name());
                    }
                }

            }
        } else {

            if (built == building.notBuilt) {

                cycleBuilt = building.notBuilt;

                cycleState = CycleState.obs_collect;

                following = true;

                built = building.built;

            }

            fullCycleOld();

        }

        if (state == autoState.finished) {
//            requestOpModeStop();
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
//            System.out.println("Y" + odometry.Y())

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

            if (!PIDToPoint){
                powerPID = new Vector2D(0,0);
            }

            if (headingAdjustment) {
                double error = targetHeading - odometry.Heading();

                if (Math.abs(odometry.getXVelocity()) < 3 && Math.abs(odometry.getYVelocity()) < 3) {
                    if (error > 0) {
                        adjustedTarget += 0.6;
                    } else {
                        adjustedTarget -= 0.6;
                    }
                } else {
                    adjustedTarget = 0;
                }

                driveBase.queueCommand(driveBase.drivePowers(new RobotPower(powerPID.getX()*0.6, powerPID.getY()*0.6, follow.getTurnPower(targetHeading + adjustedTarget, odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity()))));
            } else {
                driveBase.queueCommand(driveBase.drivePowers(new RobotPower(powerPID.getX()*0.6, powerPID.getY()*0.6, 0)));
            }

//            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0, 0, 0)));
        }

//        telemetry.addData("Y", odometry.Y());
//        telemetry.addData("Heading", odometry.Heading());
//        telemetry.addData("X", odometry.X());
//        telemetry.addData("Current command default? ", collection.getCurrentCommand() == collection.defaultCommand);
        telemetry.addData("looptime ", loopTime);
        telemetry.addData("pathing ", following);
        telemetry.addData("counter ", counter);
        telemetry.addData("vision state ", state.name());
        telemetry.addData("cycle state ", cycleState.name());
        telemetry.addData("fourbar state ", collection.getFourBarState().name());
        telemetry.addData("gripper state ", collection.getClawsState().name());
//        telemetry.addData("boolean second", Math.abs(targetHeading - odometry.Heading()));
//        telemetry.addData("", "");
        telemetry.addData("target Point", limelight.getTargetPoint());
        telemetry.update();

    }

    public void fullCycleOld(){

        //point to target for collection
        Vector2D targetExtendoPoint = new Vector2D(336, 59);

        if (cycleState == CycleState.obs_collect) {

            if (cycleBuilt == building.notBuilt) {

                //build state set
                cycleBuilt = building.built;

                //reset booleans
                following = true;
                collectSample = false;
                headingOverride = false;
                retryCollection = false;

                //set target heading
                targetHeading = 306;

                follow.setPath(paths.returnPath("collectObservationZone"));

                //reset and enable transfer fail detection
                collection.resetTransferCanceled();
                collection.setCancelTransfer(true);

                //reset and enable transfer fail detection
                collection.queueCommand(collection.preCollectNoRotate(180));

                //reset delivery gripper to transfer position
                delivery.griperRotateSev.setPosition(90);

            }

            /**
             * run collection commands
             * */
            if (!retryCollection && error(collectObservationZonePoint.getX(), odometry.X()) < 5 && error(collectObservationZonePoint.getY(), odometry.Y()) < 5 && Math.abs(odometry.Heading() - targetHeading) < 5 && !collectSample){

                // Reset booleans
                collectSample = true;

                // Queue collection and transfer
                collection.angle = 90;
                collection.queueCommand(collection.extendoTargetPoint(targetExtendoPoint));
                collection.queueCommand(collection.collect);
                collection.queueCommand(collection.transfer(Collection.tranfer.specimen));

                // Delivery rotate reset to transfer
                delivery.griperRotateSev.setPosition(90);

            } else if (collectSample && collection.getSlideTarget() == 0 && Math.abs(collection.horizontalMotor.getVelocity()) > 20){

                cycleState = CycleState.clip;
                cycleBuilt = building.notBuilt;

            } else if (collectSample && retryCollection && collection.isTransferCanceled()){

                cycleState = CycleState.clip;
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

                collection.stopTargeting();

            }

        }else if (cycleState == CycleState.clip) {

            if (cycleBuilt == building.notBuilt) {

                cycleBuilt = building.built;

                follow.setPath(paths.returnPath("goToDrob_" + state.name()));

                following = true;
                PIDToPoint = false;
                clipped = false;
                ranPreClip = false;

                targetHeading = 350;
            }

            if (!ranPreClip && collection.getCurrentCommand() == collection.defaultCommand && collection.slidesReset.isPressed() && !clipped && delivery.fourbarState == Delivery.fourBarState.transfer){
                delivery.griperRotateSev.setPosition(10);
                delivery.queueCommand(delivery.preClipBackAuto);
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
                if (follow.isFinished() && clipped && delivery.getGripperState() == Delivery.gripper.drop) {
                    state = autoState.finished;
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                    System.out.println("finished: " + state.name());
                }
            }else{
                if (follow.isFinished() && clipped && delivery.getGripperState() == Delivery.gripper.drop) {
                    state = autoState.next(state);
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                    collection.setSlideTarget(30);
                    follow.setExtendoHeading(false);
                    System.out.println("incremented state: " + state.name());
                }
            }

        }

    }

    public double error(double number1, double number2){
        return Math.abs(number1 - number2);
    }

}
