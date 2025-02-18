package org.firstinspires.ftc.teamcode.Auto.Sample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
public class Sample_full extends OpModeEX {

    pathsManager paths = new pathsManager();

    follower follow = new follower();

    double targetHeading;

    boolean drop;
    ElapsedTime dropTimer=new ElapsedTime();
    ElapsedTime rescan = new ElapsedTime();
    boolean collect = false;
    boolean autoQueued = false;
    boolean pullDownSlides = false;
    ElapsedTime dropTimerDriving = new ElapsedTime();
    boolean stop = false;
    boolean extend = false;
    boolean run8 = true;

    boolean subRetry = false;
    double adjustedTarget = 0;

    boolean headingOverride = false;
    boolean collectRetry = false;
    boolean PIDToPoint = false;
    boolean a8 = true;

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

    enum subFailsafeStates{
        visionScanning,
        collecting,
        retryCollection,
        retractingRescan,
        turningNoneFound,
        targetLargeBlock,
        finished
    }

    enum spikeStates{
        collecting,
        retractingRescan,
        visionScanning,
        collectingVision,
        depositing
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
    boolean headingAdjustment = false;

    public cycleState CycleState = cycleState.basketDrob;
    subFailsafeStates subCollectState = subFailsafeStates.visionScanning;
    spikeStates spikeState = spikeStates.collecting;

    public autoState targetState = autoState.four;
    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public building cycleBuilt = building.notBuilt;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(326, 326)),
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(305, 324))
    };

    private final sectionBuilder[] subCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(235, 290), new Vector2D(200, 236)),
    };

    private final sectionBuilder[] subCollectCloser = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(220, 295), new Vector2D(205, 236)),
    };

    private final sectionBuilder[] spikeDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(328, 328))
    };

    private final sectionBuilder[] spikeDepositSafe = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(360, 320), new Vector2D(315, 342)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(220, 280), new Vector2D(326, 335)),
    };

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initEX() {

        odometry.startPosition(344, 282, 270);

        paths.addNewPath("preloadPath");

        paths.buildPath(preloadPath);

        paths.addNewPath("collectSub");

        paths.buildPath(subCollect);

        paths.addNewPath("collectSubCloser");

        paths.buildPath(subCollectCloser);

        paths.addNewPath("dropBasket");

        paths.buildPath(subDeposit);

        paths.addNewPath("spikeTwo");

        paths.buildPath(spikeTwo);

        follow.setPath(paths.returnPath("dropBasket"));

        collection.setCancelTransfer(false);

        limelight.switchPipeline(0);
        limelight.setSortHorizontal(true);

    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (gamepad1.a){
            a8 = false;
        }

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

                limelight.setGettingResults(false);
            }

            if (delivery.slideMotor.getCurrentPosition() > 200 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                delivery.queueCommand(delivery.deposit);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(12, 12)) {
                delivery.queueCommand(delivery.deposit);

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

                targetHeading = 180;

                built = building.built;
                cycleBuilt = building.notBuilt;

                CycleState = cycleState.spikeCollect;

                PIDToPoint = true;
                pathing = false;

//                collection.setCancelTransfer(true);
                limelight.switchPipeline(0);
            }

            if (odometry.Heading() <210 && !extend){
                collection.setSlideTarget(41);
                extend = true;
            }

            if (CycleState == cycleState.spikeCollect) {

                if (cycleBuilt == building.notBuilt) {

                    cycleBuilt = building.built;
                    follow.setExtendoHeading(false);

                    collection.resetTransferCanceled();
                    collection.queueCommand(collection.collect);

                    pullDownSlides = false;
                    autoQueued = false;
                    headingOverride = false;

                    spikeState = spikeStates.collecting;
                }

                if (PIDToPoint && collection.getSlidePositionCM() < 20 && odometry.X() > 318) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(317, 340), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                switch (spikeState){
                    case collecting:

                        if (odometry.X() < 320 && !pullDownSlides) {
                            pullDownSlides = true;
                            delivery.queueCommand(delivery.deposit);
                        }

                        if (!headingAdjustment && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect && Math.abs(odometry.Heading() - targetHeading) < 7 && odometry.getXVelocity() < 15) {

                            autoQueued = true;

                            collection.angle = 90;

                            collection.queueCommand(collection.extendoTargetPoint(new Vector2D(246, 350)));

                            collection.queueCommand(collection.transfer(Collection.tranfer.spike, true));

                        }

                        if(autoQueued && collection.isTransferCanceled() && collection.getCurrentCommand() == collection.returnDefaultCommand()){

                            delivery.queueCommand(delivery.cameraScan);

                            run8 = false;

                            collection.resetTransferCanceled();
                            collection.setSlideTarget(0);

                            rescan.reset();
                            busyDetecting = false;
                            spikeState = spikeStates.retractingRescan;
                        }

                        if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140) {
                            CycleState = cycleState.basketDrob;
                            cycleBuilt = building.notBuilt;
                        }

                        break;
                    case retractingRescan:

                            if (rescan.milliseconds() > 500 && collection.getSlidePositionCM() < 5){
                                spikeState = spikeStates.visionScanning;

                                autoQueued = false;
                                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                                run8 = false;

                                busyDetecting = true;
                                detectionTimer.reset();
                                counter = 0;
                            }

                        break;
                    case visionScanning:

                        if (busyDetecting && !collect && detectionTimer.milliseconds() > (50*counter) && counter < 25){
                            counter++;
                        }

                        if (busyDetecting && !collect){

                            if (limelight.getTargetPoint() != null && counter > 12){

                                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                                delivery.overrideCurrent(true, delivery.stow);

                                delivery.runReset();

                                collect = true;
                                busyDetecting = false;

                                spikeState = spikeStates.collectingVision;
                            }

                        }

                        if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){
                            collection.queueCommand(collection.transfer(Collection.tranfer.spike));
                            autoQueued = true;
                            spikeState = spikeStates.collectingVision;
                        }

                        if (busyDetecting && counter >= 24){
                            collection.setSlideTarget(15);

                            delivery.overrideCurrent(true, delivery.stow);
                            delivery.runReset();

                            state = autoState.spikeTwo;
                            built = building.notBuilt;

                            targetHeading = 200;

                            collection.angle = 90;

                            collection.queueCommand(collection.extendoTargetPoint(new Vector2D(246, 329)));
                            collection.queueCommand(collection.collect);
                        }

                        break;
                    case collectingVision:

                        if ((collect && collection.getCurrentCommand() == collection.returnDefaultCommand() && collection.getFourBarState() == Collection.fourBar.preCollect && collection.isTransferCanceled())){
                            state = autoState.spikeTwo;
                            built = building.notBuilt;

                            collection.angle = 90;

                            targetHeading = 200;

                            collection.queueCommand(collection.extendoTargetPoint(new Vector2D(246, 327)));
                            collection.queueCommand(collection.collect);
                        }

                        if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140) {
                            CycleState = cycleState.basketDrob;
                            cycleBuilt = building.notBuilt;
                        }

                        break;
                    default:
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (PIDToPoint) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(316, 340), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                if (cycleBuilt == building.notBuilt) {
                    PIDToPoint = true;
                    cycleBuilt = building.built;

                    targetHeading = 200;

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

                    collection.queueCommand(collection.extendoTargetPoint(new Vector2D(246, 325.5)));

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

                targetHeading = 200;

                built = building.built;
                cycleBuilt = building.notBuilt;

                CycleState = cycleState.spikeCollect;
            }

            if (PIDToPoint) {
                PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(315, 340), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
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
                    busyDetecting = false;

                    counter = 0;
                    follow.setExtendoHeading(false);

                    follow.finishPath();
                    spikeState = spikeStates.collecting;
                }

                switch (spikeState){
                    case collecting:

                        if (odometry.X() < 320 && !pullDownSlides) {
                            pullDownSlides = true;
                            collection.queueCommand(collection.transfer(Collection.tranfer.spike));
                            delivery.queueCommand(delivery.depositAuto);
                        }

                        if (delivery.getSlidePositionCM() < 60 && !autoQueued) {
                            autoQueued = true;
                        }

                        if(autoQueued && collection.isTransferCanceled() && collection.getCurrentCommand() == collection.returnDefaultCommand()){
                            delivery.queueCommand(delivery.cameraScan);

                            collection.resetTransferCanceled();
                            collection.setSlideTarget(0);

                            run8 = false;

                            autoQueued = false;
                            busyDetecting = false;

                            rescan.reset();
                            spikeState = spikeStates.retractingRescan;
                        }

                        if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140 && collection.getSlideTarget() == 0) {
                            CycleState = cycleState.basketDrob;
                            cycleBuilt = building.notBuilt;
                        }

                        break;
                    case retractingRescan:
                        if (rescan.milliseconds() > 500 && collection.getSlidePositionCM() < 2){
                            spikeState = spikeStates.visionScanning;

                            autoQueued = false;
                            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                            run8 = false;

                            busyDetecting = true;
                            detectionTimer.reset();
                            counter = 0;
                        }
                        break;
                    case visionScanning:
                        if (busyDetecting && !collect && detectionTimer.milliseconds() > (50*counter) && counter < 25){
                            counter++;
                        }

                        if (busyDetecting && !collect){

                            if (limelight.getTargetPoint() != null && counter > 12){

                                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                                delivery.overrideCurrent(true, delivery.stow);
                                delivery.runReset();

                                collect = true;
                                busyDetecting = false;

                                spikeState = spikeStates.collectingVision;
                            }

                        }

                        if (busyDetecting && counter >= 24){
                            collection.setSlideTarget(15);

                            delivery.overrideCurrent(true, delivery.stow);
                            delivery.runReset();

                            state = autoState.spikeOne;
                            built = building.notBuilt;

                            collection.angle = 60;

                            collection.queueCommand(collection.extendoTargetPoint(new Vector2D(244, 302)));
                            collection.queueCommand(collection.collect);
                        }
                        break;
                    case collectingVision:

                        if (collection.getFourBarState() == Collection.fourBar.collect && !autoQueued){
                            collection.queueCommand(collection.transfer(Collection.tranfer.auto));
                            autoQueued = true;
                        }

                        if ((collection.getCurrentCommand() == collection.returnDefaultCommand() && collection.getFourBarState() == Collection.fourBar.preCollect && collection.isTransferCanceled())){
                            state = autoState.spikeOne;
                            built = building.notBuilt;

                            collection.angle = 70;

                            collection.queueCommand(collection.extendoTargetPoint(new Vector2D(244, 302)));
                            collection.queueCommand(collection.collect);
                        }

                        if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140 && collection.getSlideTarget() == 0) {
                            CycleState = cycleState.basketDrob;
                            cycleBuilt = building.notBuilt;
                        }

                        break;
                    default:
                }

            } else if (CycleState == cycleState.basketDrob) {

                if (cycleBuilt == building.notBuilt) {

                    cycleBuilt = building.built;
                    drop = true;
                    autoQueued = false;
                }

                if (PIDToPoint) {
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

                    collection.angle = 65;

                    collection.queueCommand(collection.extendoTargetPoint(new Vector2D(242.6, 299)));

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

                    targetHeading = 207;
                    follow.setExtendoHeading(false);

                    cycleBuilt = building.built;

                    PIDToPoint = true;
                    pullDownSlides = false;
                    autoQueued = false;
                    collect = false;
                    busyDetecting = false;
                    collection.setSpikeTime(2.6);

                }

                if (PIDToPoint) {
                    PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(318, 334), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                    powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                } else {
                    powerPID = new Vector2D();
                }

                switch (spikeState){
                    case collecting:
                        if (odometry.X() < 320 && !pullDownSlides) {
                            pullDownSlides = true;
                            collection.queueCommand(collection.transfer(Collection.tranfer.spike));
                            delivery.queueCommand(delivery.depositAuto);
                        }

                        if (delivery.getSlidePositionCM() < 40 && !autoQueued) {
                            autoQueued = true;
                        }

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
                        break;
                    case retractingRescan:
                        if (rescan.milliseconds() > 500 && collection.getSlidePositionCM() < 2){
                            spikeState = spikeStates.visionScanning;

                            autoQueued = false;
                            delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                            busyDetecting = true;
                            detectionTimer.reset();
                            counter = 0;
                        }
                        break;
                    case visionScanning:

                        if (busyDetecting && !collect && detectionTimer.milliseconds() > (50*counter) && counter < 25){
                            counter++;
                        }

                        if (busyDetecting && !collect){

                            if (limelight.getTargetPoint() != null && counter > 12){

                                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                                delivery.overrideCurrent(true, delivery.stow);
                                delivery.runReset();

                                collect = true;
                                busyDetecting = false;

                                spikeState = spikeStates.collectingVision;
                            }

                        }

                        if (busyDetecting && counter >= 24){
                            collection.setSlideTarget(0);

                            delivery.overrideCurrent(true, delivery.stow);
                            delivery.runReset();

                            state = autoState.one;
                            built = building.notBuilt;
                        }

                        break;
                    case collectingVision:
                        if ((collect && collection.getCurrentCommand() == collection.returnDefaultCommand() && collection.getFourBarState() == Collection.fourBar.preCollect && collection.isTransferCanceled())){
                            state = autoState.one;
                            built = building.notBuilt;

                            collection.setSlideTarget(0);
                        }

                        if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){
                            collection.queueCommand(collection.transfer(Collection.tranfer.spike));
                            autoQueued = true;
                        }

                        if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140 && collection.getSlideTarget() == 0) {
                            CycleState = cycleState.basketDrob;
                            cycleBuilt = building.notBuilt;
                        }
                        break;
                    default:
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

            if (!PIDToPoint){
                powerPID = new Vector2D();
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

        telemetry.addData("Collection command default", collection.getCurrentCommand() == collection.defaultCommand);
        telemetry.addData("Delivery command default", delivery.getCurrentCommand() == delivery.returnDefaultCommand());
        telemetry.addData("Heading error", Math.abs(targetHeading - odometry.Heading()));
        telemetry.addData("Pathing", pathing);
        telemetry.addData("Auto state", state.name());
        telemetry.addData("Spike state", spikeState.name());
        telemetry.addData("Cycle state", CycleState.name());
        telemetry.addData("", "");
        telemetry.addData("Counter", counter);
        telemetry.addData("Busy detecting", busyDetecting);
        telemetry.addData("target Point", limelight.getTargetPoint());
        telemetry.update();

    }

    public void subCycleEnum() {

        if (CycleState == cycleState.subCollect){

            if (cycleBuilt == building.notBuilt){

                //set enum states
                cycleBuilt = building.built;
                subCollectState = subFailsafeStates.visionScanning;

                //pathing code
                if (state == autoState.three || state == autoState.four){
                    follow.setPath(paths.returnPath("collectSubCloser"));
                }else{
                    follow.setPath(paths.returnPath("collectSub"));
                }

                follow.usePathHeadings(true);
                pathing = true;
                headingOverride = false;
                PIDToPoint = false;

                //reset vision and collection variables
                counter = 0;
                busyDetecting = false;
                pullDownSlides = false;
                collect = false;
                collectRetry = false;

                // transfer reset
                collection.setCancelTransfer(true);
                collection.resetTransferCanceled();

                //limelight pipeline switch
                limelight.setTargetColor(Limelight.color.yellow);
                limelight.setSortHorizontal(false);

                limelight.setReturningData(true);
            }

            switch (subCollectState){
                case visionScanning:

                    /**
                     * Pull slides down after deposit
                     * */
                    if (odometry.X() < 310 && !pullDownSlides){
                        pullDownSlides = true;
                        delivery.queueCommand(delivery.cameraScan);

                        if (collection.getFourBarState() != Collection.fourBar.preCollect){
                            collection.queueCommand(collection.collect);
                        }
                    }

                    /**
                     * Run the vision scan when the robot comes to a stop
                     * */
                    if (!busyDetecting && Math.abs(odometry.getXVelocity()) < 5 && Math.abs(odometry.getYVelocity()) < 5 && odometry.X() < 305){

                        autoQueued = false;
                        pathing = false;
                        headingOverride = true;

                        delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                        busyDetecting = true;
                        detectionTimer.reset();
                        rescan.reset();
                        counter = 0;
                        limelight.setGettingResults(true);

                        collection.resetTransferCanceled();
                    }

                    /**
                     * Running vision scan
                     * */
                    if (!collect && busyDetecting && detectionTimer.milliseconds() > (30*counter) && counter < 13 && collection.getCurrentCommand() == collection.defaultCommand && rescan.milliseconds() > 200){

                        counter++;

                        if (limelight.getTargetPoint() != null && counter > 3){

                            if (collection.getFourBarState() != Collection.fourBar.preCollect){
                                collection.queueCommand(collection.collect);
                            }

                            limelight.setGettingResults(false);

                            collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));
                            collect = true;
                            autoQueued = false;

                            subCollectState = subFailsafeStates.collecting;
                        }

                    }else if (!collect && busyDetecting && counter >= 12 && collection.getFourBarState() == Collection.fourBar.preCollect && targetHeading < 275) {

                        targetHeading = odometry.Heading() + 15;

                        headingOverride = false;
                        follow.setExtendoHeading(true);
                        subCollectState = subFailsafeStates.turningNoneFound;

                    }else if (!collect && busyDetecting && counter >= 12 && collection.getFourBarState() == Collection.fourBar.preCollect && (targetHeading+15) > 275){
                        CycleState = cycleState.basketDrob;
                        cycleBuilt = building.notBuilt;

                        delivery.setGripperState(Delivery.gripper.grab);

                        delivery.overrideCurrent(true, delivery.stow);
                        delivery.runReset();

                        collection.setSlideTarget(0);
                        collection.overrideCurrent(true, collection.stow);
                    }

                    break;
                case collecting:

                    /**
                     * stow delivery slides for transfer
                     * */
                    if (collect && collection.getSlideTarget() != 0 && delivery.slideTarget > 15){
                        delivery.overrideCurrent(true, delivery.stow);
                        delivery.runReset();
                    }

                    /**
                     * run collect
                     * */
                    if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                        collection.queueCommand(collection.transfer(Collection.tranfer.auto));

                        autoQueued = true;
                    }

                    /**
                     * if collect successful move to the depo code
                     * */
                    if (!collection.isTransferCanceled() && collection.getSlideTarget() == 0 && collection.getClawsState() == Collection.clawState.grab && delivery.getSlidePositionCM() < 15 && collect && autoQueued && collection.horizontalMotor.getVelocity() > -10 && collection.getSlidePositionCM() < 30) {
                        CycleState = cycleState.basketDrob;
                        cycleBuilt = building.notBuilt;
                    }

                    /**
                     * Check if the collect failed
                     * */
                    if (collection.isTransferCanceled() && collection.getSlidePositionCM() > 0 && collection.getSlideTarget() > 0 && collection.getFourBarState() == Collection.fourBar.preCollect){

                        collection.setSlideTarget(0);
                        headingOverride = true;

                        if (limelight.getTargetPoint() != null){
                            collection.resetTransferCanceled();
                            collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));
                            subCollectState = subFailsafeStates.retryCollection;
                            autoQueued = false;
                        }else{
                            delivery.queueCommand(delivery.cameraScan);
                            collection.resetTransferCanceled();

                            subCollectState = subFailsafeStates.retractingRescan;
                        }

                    }

                    break;
                case retryCollection:

                    if (collection.isTransferCanceled() && collection.getSlidePositionCM() > 0 && collection.getSlideTarget() > 0 && collection.getFourBarState() == Collection.fourBar.preCollect){

                        collection.setSlideTarget(0);
                        collection.turret.setPosition(collection.turretTransferPosition);
                        subRetry = true;
                        headingOverride = true;
                        run8 = false;

                        delivery.queueCommand(delivery.cameraScan);
                        collection.resetTransferCanceled();

                        subCollectState = subFailsafeStates.retractingRescan;
                        rescan.reset();
                    }

                    if (collection.getFourBarState() == Collection.fourBar.collect && collect && !autoQueued){

                        collection.queueCommand(collection.transfer(Collection.tranfer.auto));

                        autoQueued = true;

                    }

                    if (rescan.milliseconds() > 500 && !collection.isTransferCanceled() && collection.getSlideTarget() == 0 && collection.getClawsState() == Collection.clawState.grab && delivery.getSlidePositionCM() < 5 && collect && autoQueued && collection.horizontalMotor.getVelocity() > -5) {
                        CycleState = cycleState.basketDrob;
                        cycleBuilt = building.notBuilt;
                    }

                    break;
                case retractingRescan:

                    if (delivery.getCurrentCommand() == delivery.returnDefaultCommand() && delivery.getSlidePositionCM() > 16 && Math.abs(delivery.slideMotor.getVelocity()) < 5 && collection.getSlidePositionCM() < 4){
                        subCollectState = subFailsafeStates.visionScanning;

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

        } else if (CycleState == cycleState.basketDrob){

            if (cycleBuilt == building.notBuilt){

                follow.setPath(paths.returnPath("dropBasket"));

                targetHeading = 220;

                follow.usePathHeadings(false);

                cycleBuilt = building.built;

                pathing = true;
                drop = true;

                limelight.setReturningData(false);
            }

            if (collection.getCurrentCommand() == collection.defaultCommand && odometry.X() > 220 && drop){
                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;
            }

//            else if (collection.getCurrentCommand() == collection.defaultCommand && odometry.X() > 260 && !delivery.clawSensor.isPressed() && delivery.getSlidePositionCM() < 8 && delivery.fourbarState == Delivery.fourBarState.transfer) {
//                cycleBuilt = building.notBuilt;
//                CycleState = cycleState.subCollect;
//            }

            if (delivery.slideMotor.getCurrentPosition() > 655 && delivery.fourbarState == Delivery.fourBarState.transfer && drop){
                delivery.queueCommand(delivery.depositAuto);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(9,9)) {

                delivery.queueCommand(delivery.depositAuto);

                pathing = false;

                drop = false;

                dropTimerDriving.reset();
            }

            if (state == targetState){

                if (!pathing && !drop && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop){
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