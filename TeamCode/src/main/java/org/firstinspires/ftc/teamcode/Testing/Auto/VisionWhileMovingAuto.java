package org.firstinspires.ftc.teamcode.Testing.Auto;

//import com.acmerobotics.dashboard.FtcDashboard;
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

//@Autonomous
public class VisionWhileMovingAuto extends OpModeEX {

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
    boolean stopDiviving = false;
    boolean startpid = false;

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
    double offsetTimer = 0;
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
            () -> paths.addPoints(new Vector2D(200, 280), new Vector2D(200, 200)),
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

//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initEX() {

        odometry.startPosition(200, 280, 270);

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

        if (built == building.notBuilt) {
            CycleState = cycleState.subCollect;
            cycleBuilt = building.notBuilt;
            built = building.built;
        }

        subCycleEnum();

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

                follow.setHeadingLookAheadDistance(20);

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

                limelight.setReturningData(true);
                limelight.setGettingResults(true);

                offsetTimer = 0;
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

//                    if (odometry.X() < 270){
//                        follow.usePathHeadings(false);
//                        targetHeading = 270;
//                    }

                    /**
                     * Run the vision scan when the robot comes to a stop
                     * */
                    if (!busyDetecting && odometry.Y() < 250 && odometry.X() < 238 && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) < 165){

                        autoQueued = false;

                        delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                        busyDetecting = true;
                        detectionTimer.reset();
                        rescan.reset();
                        counter = 0;
                        limelight.setGettingResults(true);

                        collection.resetTransferCanceled();
                    }

                    System.out.println("Slide velocity" + collection.horizontalMotor.getVelocity());
                    System.out.println("X" + odometry.X());
                    System.out.println("Y" + odometry.Y());

                    /**
                     * Running vision scan
                     * */
                    if (counter == 4){
                        offsetTimer = detectionTimer.milliseconds() - 200;
                    }

                    if (!collect && busyDetecting && (detectionTimer.milliseconds() - offsetTimer) > (50*counter) && counter < 30){

                        System.out.println("Vision running" + counter);

                        if (odometry.Y() < 250 && Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity()) < 6 || counter < 4){
                            counter++;
                        }

                        System.out. println("Velocity" + (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())));
                        System.out.println("Target point" + limelight.getTargetPoint());

                        if (limelight.getTargetPoint() != null && counter > 3){

                            if (collection.getFourBarState() != Collection.fourBar.preCollect){
                                collection.queueCommand(collection.collect);
                            }

                            System.out.println("Targeting activated!!!");

                            limelight.setGettingResults(false);

                            pathing = false;
                            headingOverride = true;

                            collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));
                            follow.finishPath();

//                            startpid = true;
//
//                            if (startpid) {
//                                PathingPower power = follow.pidToPoint(new Vector2D(odometry.X() , odometry.Y() ), new Vector2D(odometry.X() , odometry.Y() ), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
//                                powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
//                            } else {
//                                powerPID = new Vector2D();
//                            }

                            collect = true;
                            autoQueued = false;

                            subCollectState = subFailsafeStates.collecting;
                        }

                    }else if (!collect && busyDetecting && counter >= 29 && collection.getFourBarState() == Collection.fourBar.preCollect && targetHeading < 275) {

                        targetHeading = odometry.Heading() + 15;

                        headingOverride = false;
                        follow.setExtendoHeading(true);
                        subCollectState = subFailsafeStates.turningNoneFound;

                    }else if (!collect && busyDetecting && counter >= 29 && collection.getFourBarState() == Collection.fourBar.preCollect && (targetHeading+15) > 275){
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
                    System.out.println(collection.horizontalMotor.getVelocity());

                    /**
                     * stow delivery slides for transfer
                     * */
                    if (collect && collection.getSlideTarget() != 0 && delivery.slideTarget > 15){
                        delivery.overrideCurrent(true, delivery.stow);
                        delivery.runReset();
                        stopDiviving = true;
                    }
                    /**
                     * stop driving
                     * */
                    if (stopDiviving){


                        stopDiviving = false;
                    }
                    if (startpid) {
                        PathingPower power = follow.pidToPoint(new Vector2D(odometry.X() , odometry.Y() ), new Vector2D(odometry.X() , odometry.Y() ), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                        powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
                    } else {
                        powerPID = new Vector2D();
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
                        startpid = false;
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

        }

    }
}
