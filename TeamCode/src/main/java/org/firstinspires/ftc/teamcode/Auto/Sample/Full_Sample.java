package org.firstinspires.ftc.teamcode.Auto.Sample;

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
@Autonomous(name = "Sample_full", group = "AA Comp Autos")
public class Full_Sample extends OpModeEX {

    pathsManager paths = new pathsManager();

    follower follow = new follower();

    double targetHeading;
    boolean ended = false;
    double approachAngle = 226;

    boolean drop;
    ElapsedTime dropTimer=new ElapsedTime();
    ElapsedTime rescan = new ElapsedTime();

    boolean firstApproch = true;

    ElapsedTime debuggingTimer = new ElapsedTime();
    boolean collect = false;
    boolean autoQueued = false;
    boolean pullDownSlides = false;
    ElapsedTime dropTimerDriving = new ElapsedTime();
    boolean stop = false;
    boolean run8 = true;

    boolean subRetry = false;
    double adjustedTarget = 0;
    double holdx;
    double holdy;

    boolean headingOverride = false;
    boolean collectRetry = false;
    boolean PIDToPoint = false;
    boolean stopDiviving = false;
    boolean startpid = false;
    boolean turn = true;
    double scanSpeed = 140;
    boolean slowExtend = false;

    Vector2D powerPID = new Vector2D();

    public enum autoState{
        preload,
        partnerPreload,
        spikeOne,
        spikeTwo,
        spikeThree,
        one,
        two,
        three,
        four,
        five,
        six,
        finished;

        public static autoState next(autoState current) {
            autoState[] values = autoState.values();
            int nextIndex = (current.ordinal() + 1) % values.length;
            return values[nextIndex];
        }
    }

    public enum failedSpikes{
        failed,
        worked
    }

    enum subFailsafeStates{
        visionScanning,
        collecting,
        retryCollection,
        retractingRescan,
        turningNoneFound,
        targetLargeBlock
    }

    enum spikeStates{
        collecting,
        retractingRescan
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

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    double offsetTimer = 0;
    int counter = 0;

    boolean pathing = false;
    boolean headingAdjustment = false;

    public cycleState CycleState = cycleState.basketDrob;
    subFailsafeStates subCollectState = subFailsafeStates.visionScanning;
    spikeStates spikeState = spikeStates.collecting;

    public autoState targetState = autoState.five;
    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public building cycleBuilt = building.notBuilt;

    /**
     * Fail detection on spikes
     * */
    public failedSpikes firstSpike = failedSpikes.worked;
    public failedSpikes secondSpike = failedSpikes.worked;
    public failedSpikes thirdSpike = failedSpikes.worked;

    /**
     * Paths
     * */
    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(345, 295), new Vector2D(340, 308)),
    };

    private final sectionBuilder[] depositPartnerPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(315, 330)),
    };
    //            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(326, 328)),
    private final sectionBuilder[] partnerColet = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(340, 308), new Vector2D(337, 304), new Vector2D(337, 306)),
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(300, 300))
    };

    private final sectionBuilder[] subCollectCloser = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(330, 330), new Vector2D(222, 275), new Vector2D(213, 240)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(220, 265), new Vector2D(330, 328)),
    };

//    private final sectionBuilder[] subDepositSecond = new sectionBuilder[]{
//            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(225, 260), new Vector2D(323.5, 318)),
//    };

    Vector2D spikeOneTarget = new Vector2D(242, 302.5);
    Vector2D spikeTwoTarget = new Vector2D(240, 327);
    Vector2D spikeThreeTarget = new Vector2D(240, 352.5);

    @Override
    public void initEX() {

//        odometry.startPosition(344, 282, 270);
        odometry.startPosition(335.5, 299, 225);

//        paths.addNewPath("preloadPath");
//        paths.buildPath(preloadPath);
//        paths.addNewPath("depositPartnerPath");
//        paths.buildPath(depositPartnerPath);

        paths.addNewPath("partnerCole");
        paths.buildPath(partnerColet);

        paths.addNewPath("collectSubCloser");
        paths.buildPath(subCollectCloser, 225);

        paths.addNewPath("dropBasket");
        paths.buildPath(subDeposit, 265);

        paths.addNewPath("spikeTwo");
        paths.buildPath(spikeTwo);

        follow.setPath(paths.returnPath("dropBasket"));
        collection.setCancelTransfer(false);

        limelight.switchPipeline(0);
        limelight.collectColoredSamples(false);
        limelight.setDetectionColor(true);
        limelight.setTargetColor(Limelight.color.yellow);
        limelight.setAuto(true);

    }

    @Override
    public void init_loop() {

        if (gamepad1.dpad_right){
            limelight.collectColoredSamples(true);
        }

        if (gamepad1.dpad_left){
            limelight.collectColoredSamples(false);
        }

        if (gamepad1.a){
            limelight.setDetectionColor(false);
        }

        if (gamepad1.y){
            limelight.setDetectionColor(true);
        }

        if (gamepad1.b){
            approachAngle = 226;
            scanSpeed = 140;
        }

        if (gamepad1.x){
            approachAngle = 250;
            scanSpeed = 60;
        }

        telemetry.addData("Toggle collection coloured samples", "a = false, y = true");
        telemetry.addData("Vision targeting coloured samples", limelight.getAllianceSamples());

        telemetry.addData("Set alliance color", "d_pad_right = Red, d_pad_left = Blue");
        telemetry.addData("targeting red samples", limelight.isAllianceColor());

        telemetry.addData("Collection approach angle", "y = safe, y = normal");
        telemetry.update();

        super.init_loop();
    }

    @Override
    public void loopEX() {

        if (state == autoState.preload || state == autoState.spikeOne || state == autoState.spikeTwo || state == autoState.spikeThree){
            delivery.setSpikeTransfer(true);
        }

        if (state == autoState.preload) {

            if (built == building.notBuilt) {

                collection.setOpenWide(true);
                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;

                targetHeading = 200;
                pathing = false;
                PIDToPoint = true;

                built = building.built;
                drop = true;
                dropTimer.reset();
                delivery.setSpikeTransfer(true);

//                pathing = true;
                follow.setExtendoHeading(true);
                headingOverride = false;

                collection.queueCommand(collection.collect);
                collection.setTransferType(Collection.tranfer.spike);
                limelight.setGettingResults(false);
                limelight.setReturningData(false);

                collection.setSpikeDriving(true);

                collection.setSlideTarget(32);
                collection.setTargeting(Collection.targetingTypes.spike);
            }

            if (PIDToPoint) {
                PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(322, 330), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
            } else {
                powerPID = new Vector2D();
            }

            if (delivery.getSlidePositionCM() > 0 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                delivery.queueCommand(delivery.depositAuto);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 58 && ((Math.abs(odometry.X() - 322)) < 5 && Math.abs((odometry.Y() - 330)) < 5)) {
                delivery.queueCommand(delivery.depositAuto);
//                PIDToPoint = false;
                drop = false;
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop) {
                if (state == targetState) {
                    state = autoState.finished;
                } else {
                    state = autoState.spikeOne;
                    built = building.notBuilt;
                }

            }

        } else if (state == autoState.spikeOne) {

            if (built == building.notBuilt) {
                built = building.built;
                cycleBuilt = building.notBuilt;
                CycleState = cycleState.spikeCollect;
//                PIDToPoint = true;
                collection.setSpikeDriving(false);
            }

            if (CycleState == cycleState.spikeCollect) {

                if (cycleBuilt == building.notBuilt) {

                    targetHeading = 200;
                    follow.setExtendoHeading(false);

                    cycleBuilt = building.built;

                    pathing = false;
                    PIDToPoint = true;

                    autoQueued = true;
                    pullDownSlides = false;
                    collect = false;
                    busyDetecting = false;
                    collection.setSpikeTime(0.4);

                }

                if (PIDToPoint) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(306, 330), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (odometry.X() < 314 && !pullDownSlides && !headingAdjustment){
                    delivery.queueCommand(delivery.depositAuto);
                    pullDownSlides = true;
                    collection.angle = 70;
                    collection.queueCommand(collection.extendoTargetPoint(spikeOneTarget));
                    collection.queueCommand(collection.transfer(Collection.tranfer.spike, true));
                    PIDToPoint = false;
                    headingOverride = true;
                }

//                if (odometry.X() < 330 && !pullDownSlides) {
//                    pullDownSlides = true;
//                    delivery.queueCommand(delivery.depositAuto);
//                }

//                if (collection.getCurrentCommand() == collection.defaultCommand && !autoQueued && Math.abs(targetHeading - odometry.Heading()) < 10) {
//
//                    headingOverride = true;
//
//                    collection.queueCommand(collection.preCollectNoWait);
//
//                    collection.queueCommand(collection.extendoTargetPoint(spikeOneTarget));
//
//                    collection.queueCommand(collection.collect);
//
//                    collection.queueCommand(collection.transfer(Collection.tranfer.spike));
//
//                    autoQueued = true;
//                }

                if(autoQueued && collection.isTransferCanceled() && collection.getCurrentCommand() == collection.returnDefaultCommand()){
                    delivery.queueCommand(delivery.cameraScan);
                    collection.resetTransferCanceled();
                    collection.setSlideTarget(0);
                    autoQueued = false;
                    busyDetecting = false;

                    run8 = false;

                    rescan.reset();
                    spikeState = spikeStates.retractingRescan;
                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140 && collection.getSlideTarget() == 0) {
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                    collection.setSpikeTime(2);

                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt) {
                    PIDToPoint = true;
                    cycleBuilt = building.built;
                    targetHeading = 191;
                    drop = true;

                    autoQueued = false;
                    follow.setExtendoHeading(true);
                    headingOverride = false;
                }

                if (PIDToPoint) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(317, 338), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (collection.getCurrentCommand() == collection.defaultCommand) {
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.getSlidePositionCM() > 20 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                    delivery.queueCommand(delivery.depositAuto);
                    delivery.griperRotateSev.setPosition(45);
                }

                if (collection.getCurrentCommand() == collection.defaultCommand && !autoQueued) {
                    collection.queueCommand(collection.preCollectNoWait);

                    collection.angle = 90;

                    collection.setSpikeTime(1);

                    collection.queueCommand(collection.extendoTargetPoint(spikeTwoTarget));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transfer(Collection.tranfer.spike));

                    autoQueued = true;
                }

                if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab) {
                    delivery.queueCommand(delivery.depositAuto);
                    state = autoState.spikeTwo;
                    built = building.notBuilt;
                    PIDToPoint = false;
                }

            }

        } else if (state == autoState.spikeTwo) {

            if (built == building.notBuilt) {

                targetHeading = 191;

                built = building.built;
                cycleBuilt = building.notBuilt;

                CycleState = cycleState.spikeCollect;
            }

            if (PIDToPoint) {
                PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(317, 338), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
            } else {
                powerPID = new Vector2D();
            }

            if (CycleState == cycleState.spikeCollect) {

                if (cycleBuilt == building.notBuilt) {

                    cycleBuilt = building.built;
                    collection.resetTransferCanceled();

                    pullDownSlides = false;
                    collect = false;
                    busyDetecting = false;

                    counter = 0;
                    follow.setExtendoHeading(false);

                    follow.finishPath();
                    spikeState = spikeStates.collecting;
                }

                if (odometry.X() < 330 && !pullDownSlides) {
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.depositAuto);
                    delivery.griperRotateSev.setPosition(90);
                }

//                if (collection.getCurrentCommand() == collection.defaultCommand && !autoQueued && Math.abs(targetHeading - odometry.Heading()) < 10) {
//
//                    headingOverride = true;
//
//                    collection.queueCommand(collection.preCollectNoWait);
//
//                    collection.queueCommand(collection.extendoTargetPoint(spikeTwoTarget));
//
//                    collection.queueCommand(collection.collect);
//
//                    collection.queueCommand(collection.transfer(Collection.tranfer.spike));
//
//                    autoQueued = true;
//                }

                if(autoQueued && collection.isTransferCanceled() && collection.getCurrentCommand() == collection.returnDefaultCommand()){
                    state = autoState.spikeOne;
                    built = building.notBuilt;

                    secondSpike = failedSpikes.failed;
                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140 && collection.getSlideTarget() == 0) {
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (PIDToPoint) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(316.5, 343), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (cycleBuilt == building.notBuilt) {
                    cycleBuilt = building.built;
                    drop = true;
                    targetHeading = 180;
                    autoQueued = false;
                    PIDToPoint = true;
                }

                if (collection.getCurrentCommand() == collection.defaultCommand) {
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.getSlidePositionCM() > 20 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                    delivery.queueCommand(delivery.depositAuto);
                    delivery.griperRotateSev.setPosition(45);
                }

                if (collection.getCurrentCommand() == collection.defaultCommand && !autoQueued) {
                    collection.queueCommand(collection.preCollectNoWait);

                    collection.angle = 90;

                    collection.setSpikeTime(1);

                    collection.queueCommand(collection.extendoTargetPoint(new Vector2D(spikeThreeTarget.getX(), spikeThreeTarget.getY()-12)));

                    collection.queueCommand(collection.extendoTargetPoint(spikeThreeTarget));

                    collection.queueCommand(collection.collect);

                    collection.queueCommand(collection.transfer(Collection.tranfer.spike));

                    autoQueued = true;
                }

                if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab) {
                    delivery.queueCommand(delivery.depositAuto);
                    state = autoState.spikeThree;
                    built = building.notBuilt;
                }

            }

        } else if (state == autoState.spikeThree) {

            if (built == building.notBuilt) {

                collection.setSpikeTime(0.6);
                targetHeading = 180;

                built = building.built;
                cycleBuilt = building.notBuilt;

                CycleState = cycleState.spikeCollect;
                PIDToPoint = true;
                pathing = false;
//                collection.setSlideTarget(33);

                limelight.switchPipeline(0);
            }

            if (CycleState == cycleState.spikeCollect) {

                if (cycleBuilt == building.notBuilt) {

                    cycleBuilt = building.built;
                    follow.setExtendoHeading(false);

                    collection.resetTransferCanceled();
                    collection.setCancelTransfer(false);

                    pullDownSlides = false;
                    autoQueued = false;
                    headingOverride = false;

                    spikeState = spikeStates.collecting;
                }

                if (PIDToPoint) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(315, 343), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (odometry.X() < 319 && !pullDownSlides) {
                    pullDownSlides = true;
                    delivery.queueCommand(delivery.deposit);
                    delivery.griperRotateSev.setPosition(90);
                }
//
//                else if (odometry.x) {
//
//                }

//                if (!autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect && odometry.Heading() < 184) {
//
//                    autoQueued = true;
//
//                    collection.angle = 90;
//
//                    collection.queueCommand(collection.extendoTargetPoint(spikeThreeTarget));
//
//                    collection.queueCommand(collection.transfer(Collection.tranfer.spike, true));
//
//                }

                if(autoQueued && collection.isTransferCanceled() && collection.getCurrentCommand() == collection.returnDefaultCommand()){
                    state = autoState.one;
                    built = building.notBuilt;

                    thirdSpike = failedSpikes.failed;
                    autoQueued = false;
                }

                if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140) {
                    CycleState = cycleState.basketDrob;
                    cycleBuilt = building.notBuilt;
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt) {
                    cycleBuilt = building.built;
                    PIDToPoint = true;
                    targetHeading = 200;
                    drop = true;
                    autoQueued = false;
                }

                if (PIDToPoint) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(318, 330), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (collection.getCurrentCommand() == collection.defaultCommand) {
                    delivery.slideSetPoint(delivery.autoHighBasket);
                    delivery.slides = Delivery.slideState.moving;
                }

                if (delivery.getSlidePositionCM() > 20 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                    delivery.queueCommand(delivery.depositAuto);
                    delivery.griperRotateSev.setPosition(45);
                }

//                if (collection.getCurrentCommand() == collection.defaultCommand && !autoQueued) {
//
//                    collection.queueCommand(collection.preCollectNoWait);
//
//                    collection.queueCommand(collection.extendoTargetPoint(spikeTwoTarget));
//
//                    collection.queueCommand(collection.collect);
//
//                    collection.queueCommand(collection.transfer(Collection.tranfer.spike));
//
//                    autoQueued = true;
//                }

                if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab) {
                    delivery.queueCommand(delivery.depositAuto);
                    state = autoState.one;
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                }

            }

        } else {

            if (built == building.notBuilt) {
                CycleState = cycleState.subCollect;
                cycleBuilt = building.notBuilt;
                built = building.built;
            }

            subCycleEnum();
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

//        if (autoTime.milliseconds() > 29800){
//            delivery.griperSev.setPosition(delivery.gripperDrop);
//            delivery.setGripperState(Delivery.gripper.drop);
//            ended = true;
//        }

//        System.out.println("Current time difference" + limelight.getCurrentTime());

        telemetry.addData("Collection command default", collection.getCurrentCommand() == collection.defaultCommand);
        telemetry.addData("Delivery command default", delivery.getCurrentCommand() == delivery.returnDefaultCommand());
        telemetry.addData("Heading error", Math.abs(targetHeading - odometry.Heading()));
        telemetry.addData("Pathing", pathing);
        telemetry.addData("Auto state", state.name());
        telemetry.addData("Spike state", spikeState.name());
        telemetry.addData("Cycle state", CycleState.name());
        telemetry.addData("vision state", subCollectState.name());
        telemetry.addData("Counter", counter);
        telemetry.addData("Busy detecting", busyDetecting);
        telemetry.addData("target Point", collection.isTransferCanceled());
        telemetry.update();

    }

    public void subCycleEnum() {

        if (CycleState == cycleState.subCollect){

            if (cycleBuilt == building.notBuilt){

                collection.setTransferType(Collection.tranfer.overHeadTransfer);

                //set enum states
                cycleBuilt = building.built;
                subCollectState = subFailsafeStates.visionScanning;

                follow.setPath(paths.returnPath("collectSubCloser"));

                collection.setOpenWide(false);
                follow.usePathHeadings(true);
                pathing = true;
                headingOverride = false;
                PIDToPoint = false;
                delivery.setSpikeTransfer(true);
                follow.setHeadingLookAheadDistance(40);
                slowExtend = false;

                //reset vision and collection variables
                counter = 0;
                busyDetecting = false;
                pullDownSlides = false;
                collect = false;
                collectRetry = false;
                turn = true;
                autoQueued = true;
                startpid = false;
                follow.setExtendoHeading(false);

                // transfer reset
                collection.setCancelTransfer(true);
                collection.resetTransferCanceled();

                //limelight pipeline switch
                limelight.setTargetColor(Limelight.color.yellow);

                limelight.setReturningData(true);
                limelight.setGettingResults(true);
                collection.setTargeting(Collection.targetingTypes.slower);
                collection.setResettingDisabled(false);

                offsetTimer = 0;
                debuggingTimer.reset();

                powerPID = new Vector2D();
                collection.setSpikeDriving(true);
            }

            System.out.println("Delivery = default" + (delivery.getCurrentCommand() == delivery.returnDefaultCommand()));
            System.out.println("Delivery = camera" + (delivery.getCurrentCommand() == delivery.cameraScan));

            switch (subCollectState){

                case visionScanning:

                    /**
                     * Pull slides down after deposit
                     * */
                    if (odometry.X() < 300 && !pullDownSlides){
                        pullDownSlides = true;
                        delivery.queueCommand(delivery.cameraScan);
                        delivery.griperRotateSev.setPosition(90);

                        System.out.println("Pulled down slides" + debuggingTimer.milliseconds());

                        if (collection.getFourBarState() != Collection.fourBar.preCollect){
                            collection.overrideCurrent(true, collection.collect);
                            collection.targetPositionManuel = new Vector2D(6, 20);
                            collection.armEndPointIncrement(-15, -10, false);
                        }
                    }

                    if (odometry.X() < 240 && turn){
                        follow.usePathHeadings(false);
                        targetHeading = approachAngle;
                        turn = false;
//                        System.out.println("Set angle of approach" + debuggingTimer.milliseconds());
                    }


                    /**
                     * Run the vision scan when the robot comes to a stop
                     * */
//                    if (!busyDetecting && follow.isFinished(5,5) && (Math.abs(odometry.Heading() - targetHeading) < 6 || headingOverride)){

                    if (!busyDetecting && odometry.Y() < 269 && odometry.X() < 230 && (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())) < scanSpeed && (Math.abs(targetHeading - odometry.Heading()) < 8 || headingOverride)){

                        autoQueued = false;

                        busyDetecting = true;
                        detectionTimer.reset();
                        rescan.reset();
                        counter = 0;

                        limelight.setGettingResults(true);
                        limelight.setReturningData(true);

                        pathing = false;
                        headingOverride = true;
                        PIDToPoint = false;

                        firstApproch = true;

                        targetHeading = odometry.Heading() - 5;

                        System.out.println("Activated vision: " + debuggingTimer.milliseconds());

//                        System.out.println("Y Velocity: " + );
//                        follow.usePathHeadings(false);

                        collection.resetTransferCanceled();
                        holdx = 0;
                        holdy = 0;
                    }

//                    System.out.println("Slide velocity" + collection.horizontalMotor.getVelocity());
//                    System.out.println("X" + odometry.X());
//                    System.out.println("Y" + odometry.Y());

//                    System.out.println("Robot velocity: " + (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())));

                    /**
                     * Running vision scan
                     * */
                    if (counter == 4){
                        offsetTimer = detectionTimer.milliseconds() - 200;
                    }

                    if (!collect && busyDetecting && (detectionTimer.milliseconds()) > (50*counter) && counter < 10){

//                        System.out.println("Vision running" + debuggingTimer.milliseconds());

                        if (odometry.Y() < 250 && Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity()) < 10 || counter < 4){
                            counter++;
                        }

//                        System.out. println("Velocity" + (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())));

                        if (limelight.getTargetPoint() != null && counter > 1){

                            System.out.println("Targeting activated!!!");

                            if (collection.getFourBarState() != Collection.fourBar.preCollect){
                                collection.queueCommand(collection.collect);
                            }

//                            holdy = odometry.Y();
//                            holdx = odometry.X();
//                            targetHeading = odometry.Heading();
//                            follow.usePathHeadings(false);

//                            targetHeading = odometry.Heading();
                            limelight.setGettingResults(false);
                            limelight.setReturningData(false);

                            pathing = false;
                            headingOverride = true;

                            System.out.println("Target point X" + limelight.getTargetPoint().getTargetPoint().getX());
                            System.out.println("Target point Y" + limelight.getTargetPoint().getTargetPoint().getY());

                            collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));
                            follow.finishPath();

                            if (limelight.getTargetPoint() == null && approachAngle < 258 && firstApproch){
                                approachAngle += 15;
                                scanSpeed -= 60;
                                if (scanSpeed < 0){
                                    scanSpeed = 0;
                                }

                                firstApproch = false;
                            }

                            startpid = true;
                            collect = true;
                            autoQueued = false;

                            subCollectState = subFailsafeStates.collecting;
                        }

                    }else if (!collect && busyDetecting && counter >= 9 && collection.getFourBarState() == Collection.fourBar.preCollect && targetHeading < 265) {

                        targetHeading = odometry.Heading() + 20;
                        approachAngle += 10;

                        headingOverride = false;
                        follow.setExtendoHeading(true);
                        subCollectState = subFailsafeStates.turningNoneFound;

                    }else if (!collect && busyDetecting && counter >= 9 && collection.getFourBarState() == Collection.fourBar.preCollect && (targetHeading+15) > 265){
                        CycleState = cycleState.basketDrob;
                        cycleBuilt = building.notBuilt;

                        delivery.setGripperState(Delivery.gripper.grab);

                        delivery.overrideCurrent(true, delivery.stow);
                        delivery.runReset();

                        collection.stopTargeting();
                        collection.setSlideTarget(0);
                        collection.overrideCurrent(true, collection.stow);
                    }

                    break;
                case collecting:

                    /**
                     * stow delivery slides for transfer
                     * */
                    if (collect && collection.getSlideTarget() != 0 && delivery.slideTarget > 15){
                        delivery.overrideCurrent(true, delivery.spike);
                        stopDiviving = true;
                    }

                    /**
                     * stop driving
                     * */
//                    if (stopDiviving){
//                        stopDiviving = false;
//                    }
//
//                    if (startpid) {
//                        PathingPower power = follow.pidToPoint(new Vector2D(odometry.X() , odometry.Y()), new Vector2D(holdx, holdy), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
//                        powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
//                    } else {
//                        powerPID = new Vector2D();
//                    }

                    /**
                     * run collect
                     * */
                    if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                        collection.queueCommand(collection.transfer(Collection.tranfer.overHeadTransfer));

                        autoQueued = true;
                    }

                    if (collection.getFourBarState() == Collection.fourBar.preCollect && collection.getCurrentCommand() == collection.defaultCommand && !collection.isTransferCanceled()){
                        collection.stopTargeting();
                        collection.targetPositionManuel = new Vector2D(6, 20);
                        collection.armEndPointIncrement(-15, -10, false);
                        headingOverride = true;

                        if (limelight.getTargetPoint() != null){
                            collection.resetTransferCanceled();
                            collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));
                            subCollectState = subFailsafeStates.retryCollection;
                            autoQueued = false;
                        }else{
                            delivery.queueCommand(delivery.cameraScan);
                            System.out.println("Delivery = default (Out of range)" + (delivery.getCurrentCommand() == delivery.returnDefaultCommand()));
                            collection.resetTransferCanceled();

                            subCollectState = subFailsafeStates.retractingRescan;
                        }
                    }

                    /**
                     * if collect successful move to the depo code
                     * */
                    if (!collection.isTransferCanceled() && collection.getSlideTarget() == 0 && collection.getClawsState() == Collection.clawState.grab && collect && autoQueued && collection.getSlidePositionCM() < 40 && collection.abortTimer.milliseconds() > 400) {
                        CycleState = cycleState.basketDrob;
                        cycleBuilt = building.notBuilt;
                        startpid = false;
                    }

                    /**
                     * Check if the collect failed
                     * */
                    if (collection.isTransferCanceled() && collection.getFourBarState() == Collection.fourBar.preCollect){
                        collection.stopTargeting();
                        collection.targetPositionManuel = new Vector2D(6, 20);
                        collection.armEndPointIncrement(-15, -10, false);
                        headingOverride = true;

                        if (limelight.getTargetPoint() != null){
                            collection.resetTransferCanceled();
                            collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));
                            subCollectState = subFailsafeStates.retryCollection;
                            autoQueued = false;
                        }else{
                            delivery.queueCommand(delivery.cameraScan);
                            System.out.println("Delivery = default (Queued scan)" + (delivery.getCurrentCommand() == delivery.returnDefaultCommand()));
                            collection.resetTransferCanceled();

                            subCollectState = subFailsafeStates.retractingRescan;
                        }

                    }

                    break;
                case retryCollection:

                    if (collect && collection.getSlideTarget() != 0 && delivery.slideTarget > 15){
                        delivery.overrideCurrent(true, delivery.spike);
//                        delivery.runReset();
                        stopDiviving = true;
                    }

                    if (collection.isTransferCanceled() && collection.getFourBarState() == Collection.fourBar.preCollect){
                        collection.stopTargeting();
                        collection.targetPositionManuel = new Vector2D(6, 20);
                        collection.armEndPointIncrement(-15, -10, false);
                        subRetry = true;
                        headingOverride = true;
                        run8 = false;

                        System.out.println("(Queued scan)");
                        delivery.queueCommand(delivery.cameraScan);
                        collection.resetTransferCanceled();

                        subCollectState = subFailsafeStates.retractingRescan;
                        rescan.reset();
                    }

                    if (collection.getFourBarState() == Collection.fourBar.preCollect && collection.getCurrentCommand() == collection.defaultCommand && !collection.isTransferCanceled()){
                        collection.stopTargeting();
                        collection.targetPositionManuel = new Vector2D(6, 20);
                        collection.armEndPointIncrement(-15, -10, false);
                        headingOverride = true;

                        delivery.queueCommand(delivery.cameraScan);
                        System.out.println("Delivery = default (Out of range)" + (delivery.getCurrentCommand() == delivery.returnDefaultCommand()));
                        collection.resetTransferCanceled();

                        subCollectState = subFailsafeStates.retractingRescan;

                    }

                    if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                        collection.queueCommand(collection.transfer(Collection.tranfer.overHeadTransfer));

                        autoQueued = true;

                    }

                    if (rescan.milliseconds() > 500 && !collection.isTransferCanceled() && collection.getSlideTarget() == 0 && collection.getClawsState() == Collection.clawState.grab && collect && autoQueued && collection.abortTimer.milliseconds() > 400) {
                        CycleState = cycleState.basketDrob;
                        cycleBuilt = building.notBuilt;
                    }

                    break;
                case retractingRescan:

//                    System.out.println("Delivery = default" + (delivery.getCurrentCommand() == delivery.returnDefaultCommand()));

                    if (delivery.getCurrentCommand() == delivery.returnDefaultCommand() && delivery.getSlidePositionCM() > 16 && Math.abs(delivery.slideMotor.getVelocity()) < 5 && collection.getSlidePositionCM() < 4){
                        subCollectState = subFailsafeStates.visionScanning;
                        System.out.println("Delivery = default" + (delivery.getCurrentCommand() == delivery.returnDefaultCommand()));

                        busyDetecting = false;
                        counter = 0;
                        collect = false;
                        collection.resetTransferCanceled();
                    }

                    break;
                case turningNoneFound:

                    if (!headingAdjustment){
                        collect = false;
                        headingOverride = true;
                        busyDetecting = false;
                        counter = 0;
                        run8 = false;

                        subCollectState = subFailsafeStates.visionScanning;
                    }

                    break;
                case targetLargeBlock:
                    break;
                default:
            }

            System.out.println("Vision scan" + subCollectState.name());

        } else if (CycleState == cycleState.basketDrob){

            if (cycleBuilt == building.notBuilt){

                follow.setPath(paths.returnPath("dropBasket"));

                if (state == autoState.four){
                    targetHeading = 206;
                } else if (state == autoState.five) {
                    targetHeading = 204;
                }else {

                }

                targetHeading = 212;
                odometry.offsetY(1.7);

                follow.setHeadingOffset(0);

                follow.usePathHeadings(false);

                cycleBuilt = building.built;

                pathing = true;
                drop = true;

                limelight.setReturningData(false);
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && odometry.X() > 145 && drop){
                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;
                if (state != autoState.five){
                    collection.setSlideTarget(42);
                }
                slowExtend = true;
            }

//            if (slowExtend && collection.getSlidePositionCM() < 30)

//            else if (collection.getCurrentCommand() == collection.defaultCommand && odometry.X() > 260 && !delivery.clawSensor.isPressed() && delivery.getSlidePositionCM() < 8 && delivery.fourbarState == Delivery.fourBarState.transfer) {
//                cycleBuilt = building.notBuilt;
//                CycleState = cycleState.subCollect;
//            }

            if (delivery.slideMotor.getCurrentPosition() > 230 && delivery.fourbarState == Delivery.fourBarState.transfer && drop){
                delivery.queueCommand(delivery.depositAuto);
            }

            if (state == targetState){

                if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(18,18)) {

                    delivery.queueCommand(delivery.depositAuto);

                    pathing = false;

                    drop = false;

                    dropTimerDriving.reset();
                }

                if (!pathing && !drop && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop && !ended){
                    follow.setPath(paths.returnPath("spikeTwo"));
                    pathing = true;
                    stop = false;
                } else if (!stop && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop && odometry.X() < 310) {
                    stop = true;
                    delivery.queueCommand(delivery.depositAuto);
                }

                if (stop && delivery.getSlidePositionCM() < 5) {
                    state = autoState.finished;
                }

            }else{

                if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(18,18)) {

                    delivery.queueCommand(delivery.depositAuto);

                    pathing = false;
                    state = autoState.next(state);
                    built = building.notBuilt;
                }

            }

        }

    }
}
