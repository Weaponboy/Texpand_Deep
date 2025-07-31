package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.Execute;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.command_library.Hardware.TargetSample;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class Collection extends SubSystem {

    Delivery delivery = new Delivery(getOpModeEX());

    boolean hangHold = false;
    boolean runningCheck = false;

    public ElapsedTime abortTimer = new ElapsedTime();
    double abortTime = 0;
    boolean transferToFar = false;

    public boolean disableOutOfRangeDetection = false;

    // slides
    public MotorEx horizontalMotor = new MotorEx();
    double extendoPower = 0;
    boolean transferRetryBoolean = true;

    public void setSpikeTime(double spikeTime) {
        this.spikeTime = spikeTime;
    }

    // 1.8 is safe speed
    public double spikeTime = 1.9;
    boolean spikeDriving = false;

    //servos
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot = new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoDegrees gripServo = new ServoDegrees();
    public ServoDegrees turret = new ServoDegrees();

    // sensors
    public AxonEncoder turretPosition = new AxonEncoder();
    public TouchSensor breakBeam;
    public TouchSensor slidesReset;

    /**
     * servo time per degrees
     * */
    double microRoboticTime = (double) 500 / 360;
    double gripperOpenTime = 200;

    //robot and arm size
    double robotLength = 35 * 0.5;
    double clawOffsetFromSlides = 20;

    boolean runSet = false;
    boolean TransferDrop = false;
    boolean runSlideSet = false;

    ElapsedTime WaitForTranferDrop = new ElapsedTime();
    ElapsedTime secondPivotWaitTimer = new ElapsedTime();

    /**states*/
    public enum fourBar{
        preCollect,
        collect,
        visionScan,
        transferringStates,
        preClipLow,
        stowedChamber,
        transferUp,
        stowed,
        transferInt
    }

    public enum clawState{
        drop,
        grab,
        slightRelease,
        griperHang,
        openFull
    }

    public enum tranfer {
        spike,
        teleop,
        subCycles,
        underChamberCycle,
    }

    /**
     * collect position values
     * */
    double mainPivotCollect = 89;
    double secondPivotCollect = 322;

    /**
     * preCollect position values
     * */
    double mainPivotPreCollect = 108;
    double secondPivotPreCollect = 314;

    /**
     * preCollect position values
     * */
    double mainPivotMidTransfer = 180;
    double secondPivotMidTransfer = 160;

    /**
     * stowed position values
     * */
    double mainPivotStow = 165;
    double secondPivotStow = 152;
    public double turretTransferPosition = 167.5;

    /**
     * stowed position values
     * */
    double mainPivotChamberStowed = 195;
    double secondPivotChamberStowed = 190;

    /**
     * stowed position values
     * */
    double mainPivotTransferSlam = 184;
    double secondPivotTransferSlam = 132;

    /**
     * stowed position values
     * */
    double mainPivotTransferAuto = 188;
    double secondPivotTransferAuto = 132;

    /**
     * stowed position values
     * */
    double mainPivotTransferAutoSpike = 190;
    double secondPivotTransferAutoSpike = 161;

    /**
     * stowed position values
     * */
    double mainPivotTransferAutoSpikeDriving = 200;
    double secondPivotTransferAutoSpikeDriving = 154;

//    /**
//     * stowed position values
//     * */
//    double mainPivotTransfer = 198;
//    double secondPivotTransfer = 148;
    double rotateTransfer = 90;

    public double mainPivotHang = 83;
    public double secondPivotHang = 210;

    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;

    boolean autoCollecting = false;
    double slidesWaitTime = 0;

    /**enum states*/
    private fourBar fourBarState = fourBar.stowed;
    private fourBar fourBarTargetState = fourBar.stowed;
    private clawState clawsState = clawState.drop;
    private tranfer transferType = tranfer.teleop;

    private tranfer transferTypeSaved = tranfer.teleop;
    boolean resetTransfer = false;

    /**PID controllers**/
    PIDController adjustment = new PIDController(0.018, 0, 0.06);
    PIDController adjustmentClose = new PIDController(0.014, 0, 0.009);

    /**Slide target stuff**/
    double slideTarget;
    boolean longTarget = false;

    public double turretTargetPosition = 0;
    boolean transferCanceled = false;
    boolean cancelTransfer = true;
    int transferCounter = 0;

    public Vector2D targetPosition = new Vector2D(clawOffsetFromSlides, clawOffsetFromSlides);
    public Vector2D targetPositionManuel = new Vector2D(clawOffsetFromSlides, clawOffsetFromSlides);

    /**gripper positions*/
    double gripperDrop = 92;
    double gripperGrab = 57;
    double gripperHangGrab = 94;
    double gripperSlightRelease = 40;
    double gripperOpenFull = 105;

    public void setOpenWide(boolean openWide) {
        this.openWide = openWide;
    }

    public boolean isOpenWide() {
        return openWide;
    }

    boolean openWide = false;

    boolean braking = false;
    ElapsedTime brakingTimer = new ElapsedTime();

    public boolean resettingSlides = false;

    RobotPower RobotPosition = new RobotPower();

    boolean angleRecheck = true;
    boolean goPositive = false;
    boolean goNegative = false;
    boolean over = false;
    public double angle;
    public double parallelAngle;
    public double manualAngle = 0;

    boolean keepTargeting = false;
    boolean exitTargeting = false;

    public enum targetingTypes{
        spike,
        normal,
        slower
    }

    public void setResettingDisabled(boolean resettingDisabled) {
        this.resettingDisabled = resettingDisabled;
    }

    boolean transferSuccessful = false;

    boolean resettingDisabled = false;
    targetingTypes targeting = targetingTypes.normal;

    public Collection(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }

    double IErrorCorrection = 0;

    public void updateDelivery(Delivery delivery){
        this.delivery = delivery;
    }

    @Override
    public void init() {
        horizontalMotor.initMotor("horizontalMotor", getOpModeEX().hardwareMap);

        horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fourBarMainPivot.initServo("fourBarMainPivot", getOpModeEX().hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot", getOpModeEX().hardwareMap);
        griperRotate.initServo("gripperRotate", getOpModeEX().hardwareMap);
        gripServo.initServo("gripServo", getOpModeEX().hardwareMap);
        turret.initServo("linearRailServo", getOpModeEX().hardwareMap);

        turretPosition.init(getOpModeEX().hardwareMap, "turretFeedback");
        breakBeam = getOpModeEX().hardwareMap.get(TouchSensor.class, "clawsensor");
        slidesReset = getOpModeEX().hardwareMap.get(TouchSensor.class, "CollectionReset");

        turretPosition.setOffset(-14);

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(335);
        turret.setRange(335);

        turret.setOffset(-7.5);
        fourBarMainPivot.setOffset(4);
        fourBarSecondPivot.setOffset(-5);

        turret.setDirection(Servo.Direction.REVERSE);

        turret.setPosition(167.5);

        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);
        gripServo.setRange(180);

//        closeAim = getOpModeEX().hardwareMap.get(WebcamName.class, "webcam");
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(closeAim);
//        builder.addProcessor(sampleDetector);
//        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//        builder.setCameraResolution(new Size(1280, 960));
//        portal = builder.build();

        griperRotate.setDirection(Servo.Direction.REVERSE);

        //positive = left from the top
        griperRotate.setOffset(-8);
        griperRotate.setPosition(90);

        setClawsState(clawState.drop);
        gripServo.setPosition(gripperDrop);

        Stowed.execute();
        runReset();
    }

    public void setHangHold(boolean hangHold) {
        this.hangHold = hangHold;
    }

    @Override
    public void execute() {

        executeEX();

        double ticksPerCM = (double) 190 / 18;
        double error;

        if (hangHold) {
            horizontalMotor.update(0);
        } else {

            error = Math.abs((slideTarget*ticksPerCM) - (double) horizontalMotor.getCurrentPosition());

            if (!resettingDisabled && !resettingSlides && !slidesReset.isPressed() && slideTarget == 0 && Math.abs(horizontalMotor.getVelocity()) < 10 && horizontalMotor.getCurrentPosition() < 100){
                slideTarget = 0;
                extendoPower = -0.55;
                resettingSlides = true;
            }else if (resettingSlides && slidesReset.isPressed()){
                slideTarget = 0;
                extendoPower = -0.04;
                resettingSlides = false;

                horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (slidesReset.isPressed() && slideTarget == 0){
                extendoPower = -0.04;
            }else if (error > 2 && !resettingSlides){

                double notAbsError = (slideTarget * ticksPerCM) - horizontalMotor.getCurrentPosition();

                IErrorCorrection = 0;

                if (longTarget){
                    extendoPower = Range.clip(adjustment.calculate(((slideTarget + IErrorCorrection) * ticksPerCM), horizontalMotor.getCurrentPosition()), -1, 1);
                }else{
                    extendoPower = Range.clip(adjustmentClose.calculate(((slideTarget + IErrorCorrection) * ticksPerCM), horizontalMotor.getCurrentPosition()), -1, 1);
                }

            }else if (slidesReset.isPressed()) {
                extendoPower = 0;
                if (slideTarget == 0) {
                    resettingSlides = false;
                }
            }

            if (error < 10 && longTarget && slideTarget != 0){
                longTarget = false;
            }else if (slideTarget == 0 && horizontalMotor.getVelocity() < 10 && longTarget){
                longTarget = false;
            }

            if (clawsState == clawState.grab){
                gripServo.setPosition(gripperGrab);
            } else if (clawsState == clawState.drop){
                if (openWide) {
                    gripServo.setPosition(gripperOpenFull);
                }else {
                    gripServo.setPosition(gripperDrop);
                }
            } else if (clawsState == clawState.slightRelease){
                gripServo.setPosition(gripperSlightRelease);
            }else if (clawsState == clawState.openFull){

            }else if (clawsState == clawState.griperHang){
                gripServo.setPosition(gripperHangGrab);
            }

            if (isTransferCanceled() && fourBarState != fourBar.preCollect){
                transferCanceled = false;
            }

            horizontalMotor.update(Range.clip(extendoPower, -1, 1));

            if (keepTargeting && getCurrentCommand() != extendoTargetPoint) {
                double slidetargot = calculateKinematicsGlobal();
                if (slidetargot != 18763) {
                    setSlideTarget(slidetargot);
                }
            }
        }
    }

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    private final Command preCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPreCollect);
                fourBarSecondPivot.setPosition(secondPivotPreCollect);
            }
    );

    private final Command preCollectLowerSpikes = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPreCollect-7);
                fourBarSecondPivot.setPosition(secondPivotPreCollect + 6);
            }
    );

    private final Command hang = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotHang);
                fourBarSecondPivot.setPosition(secondPivotHang);
            }
    );

    private final Command prePreCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                fourBarSecondPivot.setPosition(secondPivotPreCollect);
            }
    );

    private final Command Collect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotCollect);
                fourBarSecondPivot.setPosition(secondPivotCollect);
            }
    );

    private final Command TransferSlam = new Execute(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotTransferSlam);
                fourBarMainPivot.setPosition(mainPivotTransferSlam);

                clawsState = clawState.grab;
            }
    );

    private final Command TransferAuto = new Execute(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotTransferAuto);
                fourBarMainPivot.setPosition(mainPivotTransferAuto);

                clawsState = clawState.grab;
            }
    );

    private final Command TransferAutoSpike = new Execute(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotTransferAutoSpike);
                fourBarMainPivot.setPosition(mainPivotTransferAutoSpike);
                clawsState = clawState.grab;
            }
    );

    private final Command TransferAutoSpikeDriving = new Execute(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotTransferAutoSpikeDriving);
                fourBarMainPivot.setPosition(mainPivotTransferAutoSpikeDriving);
                clawsState = clawState.grab;
            }
    );

    public final Command Stowed = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotStow);
                fourBarSecondPivot.setPosition(secondPivotStow);
            }
    );

    public Command transferNoSave(tranfer transferType){
        resetTransfer = true;
        transferTypeSaved = this.transferType;
        this.transferType = transferType;
        return transfer;
    }

    public Command transfer(tranfer transferType){
        this.transferType = transferType;
        return transfer;
    }

    public Command transfer(tranfer transferType, boolean queueArmDown){
        this.transferType = transferType;
        queueCommand(collect);
        return transfer;
    }

    public final Command transferDropSlam = new LambdaCommand(
            () -> {
                TransferSlam.execute();
                setClawsState(clawState.slightRelease);
                WaitForTranferDrop.reset();

                TransferDrop = false;
                fourBarState = fourBar.stowed;
            },
            () -> {
                setClawsState(clawState.slightRelease);
                if (WaitForTranferDrop.milliseconds() > 60){
                    TransferDrop = true;
                }
            },
            () -> TransferDrop

    );

    public final Command transferDropAuto = new LambdaCommand(
            () -> {
//                TransferAuto.execute();
//                setClawsState(clawState.slightRelease);
                WaitForTranferDrop.reset();

                TransferDrop = false;
                fourBarState = fourBar.stowed;
            },
            () -> {
//                setClawsState(clawState.slightRelease);
                if (WaitForTranferDrop.milliseconds() > 40){
                    TransferDrop = true;
                }
            },
            () -> TransferDrop

    );

    public final Command openGripper = new LambdaCommand(
            () -> {
                setClawsState(clawState.drop);
                Stowed.execute();
                fourBarTimer.reset();
            },
            () -> {
            },
            () -> fourBarTimer.milliseconds() > 60
    );

    public final Command retryHighTeleFirst = new LambdaCommand(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotTransferAutoSpike + 30);
                fourBarMainPivot.setPosition(mainPivotTransferAutoSpike - 30);
                setClawsState(clawState.grab);
                WaitForTranferDrop.reset();

                TransferDrop = false;
                fourBarState = fourBar.preClipLow;
            },
            () -> {

                fourBarSecondPivot.setPosition(secondPivotTransferAutoSpike + 30);
                fourBarMainPivot.setPosition(mainPivotTransferAutoSpike - 30);

                if (WaitForTranferDrop.milliseconds() > 400){
                    TransferDrop = true;
                }
            },
            () -> TransferDrop

    );

    public final Command retryHighTele = new LambdaCommand(
            () -> {
                TransferAutoSpike.execute();
                setClawsState(clawState.grab);
                WaitForTranferDrop.reset();

                TransferDrop = false;
                fourBarState = fourBar.transferUp;
            },
            () -> {
                if (WaitForTranferDrop.milliseconds() > 400){
                    TransferDrop = true;
                }
            },
            () -> TransferDrop

    );

    public final Command openGripperRetryTeleHigh = new LambdaCommand(
            () -> {
                transferSuccessful = false;
                abortTimer.reset();
                cancelTransfer = false;
            },
            () -> {

                if ((!transferSuccessful && delivery.getGripperState() == Delivery.gripper.grab && delivery.clawSensor.isPressed() || !transferRetryBoolean) && abortTimer.milliseconds() > 100){
                    setClawsState(clawState.drop);
                    transferSuccessful = true;
                    abortTimer.reset();
                }else if (delivery.getGripperState() == Delivery.gripper.grab && !delivery.clawSensor.isPressed() && abortTimer.milliseconds() > 200 && !transferSuccessful){
                    clearQueue();

                    queueCommand(delivery.openGripper);

                    queueCommand(retryHighTeleFirst);

                    queueCommand(retryHighTele);

                    queueCommand(delivery.closeGripperSpike);

                    queueCommand(openGripper);

                    cancelTransfer = true;
                }

                if (transferSuccessful && abortTimer.milliseconds() > 100){
                    Stowed.execute();
                }

            },
            () -> transferSuccessful && abortTimer.milliseconds() > 140 || cancelTransfer
    );

    public Command collect = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCollect)*1, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotCollect)*0.5);
                    transferWaitTime = 40;

                    Collect.execute();

                } else if (fourBarState == fourBar.collect) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotPreCollect)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*3);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;

                    preCollect.execute();

                }else if (fourBarState == fourBar.stowed){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotPreCollect)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*3);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    prePreCollect.execute();

                }else if (!(fourBarState == fourBar.transferringStates)){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotPreCollect)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*3);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;

                    if (getTransferType() == Collection.tranfer.spike){
                        preCollectLowerSpikes.execute();
                    }else {
                        preCollect.execute();
                    }

                    griperRotate.setPosition(rotateTransfer);

                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> (fourBarState == fourBar.preCollect) || (fourBarState == fourBar.collect)
    );

    public Command visionScan = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState != fourBar.visionScan){

                    fourBarTimer.reset();
                    fourBarState = fourBar.visionScan;
                    transferWaitTime = 0;

                    clawsState = clawState.drop;
                    fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                    fourBarSecondPivot.setPosition(secondPivotPreCollect);

                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> (fourBarState == fourBar.visionScan)
    );

    public Command StowForHang = new LambdaCommand(
            () -> {
                fourBarState = fourBar.preCollect;
            },
            () -> {

                if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*3);

                    Stowed.execute();
                    gripServo.setPosition(gripperGrab + 10);
                    griperRotate.setPosition(45);

                } else if (fourBarState == fourBar.transferInt) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.abs(turret.getPositionDegrees()- 35)*4;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    turret.setPosition(35);

                }else if (fourBarState == fourBar.transferUp){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotHang)*3, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotHang)*3);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowedChamber;

                    hang.execute();
//                    fourBarSecondPivot.setPosition(secondPivotHang-40);

                }else if (fourBarState == fourBar.stowedChamber) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.abs(turret.getPositionDegrees()- 56)*4;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    turret.setPosition(56);
                    hang.execute();

                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> false
    );

    public Command preCollectNoWait = new LambdaCommand(
            () -> {},
            () -> {
                fourBarState = fourBar.preCollect;
                preCollect.execute();
            },
            () -> fourBarState == fourBar.preCollect
    );

    public Command extendoTargetPoint(Vector2D targetPoint){
        targetPosition = targetPoint;
        return extendoTargetPoint;
    }

    public Command extendoTargetPoint = new LambdaCommand(
            () -> {
                abortTimer.reset();

                angleRecheck = true;
                runSet = false;
                abortTime = 1000;
            },
            () -> {

                keepTargeting = true;

                double newSlideTarget = calculateKinematicsGlobal();
//                System.out.println("SLIDE TARGET" + newSlideTarget);

                if (newSlideTarget != 18763){
                    setSlideTarget(newSlideTarget);
//                    System.out.println("RAN SET IN EXTENDO TARGETING" + runSet);
                    runSet = true;
                }

                if (targeting == targetingTypes.spike){
                    exitTargeting = runSet && Math.abs(getSlideTarget() - getSlidePositionCM()) < 5 && Math.abs(horizontalMotor.getVelocity()) < 120 && Math.abs(turret.getPositionDegrees() - turretPosition.getPosition()) < 8 || !runSet && abortTimer.milliseconds() > abortTime;
                }else if (targeting == targetingTypes.normal){
                    exitTargeting = runSet && Math.abs(getSlideTarget() - getSlidePositionCM()) < 3 && Math.abs(horizontalMotor.getVelocity()) < 120 && Math.abs(turret.getPositionDegrees() - turretPosition.getPosition()) < 8 || !runSet && abortTimer.milliseconds() > abortTime;
                } else if (targeting == targetingTypes.slower) {
                    exitTargeting = runSet && Math.abs(getSlideTarget() - getSlidePositionCM()) < 1.5 && Math.abs(horizontalMotor.getVelocity()) < 45 && Math.abs(turret.getPositionDegrees() - turretPosition.getPosition()) < 4 || !runSet && abortTimer.milliseconds() > abortTime;

//                    System.out.println("Slides" + (Math.abs(getSlideTarget() - getSlidePositionCM()) < 2 && Math.abs(horizontalMotor.getVelocity()) < 60));
//                    System.out.println("Turret" + (Math.abs(turretTargetPosition - turretPosition.getPosition()) < 6));
                }

                if ((runSet || abortTimer.milliseconds() > 100) && newSlideTarget == 18763 && !disableOutOfRangeDetection){
                    exitTargeting = true;
                    clearQueue();
//                    System.out.println("Out of range excited targeting");
                }

            },
            () -> exitTargeting
    );

    public Command autoCollectGlobal(TargetSample targetPoint){
        targetPosition = targetPoint.getTargetPoint();
        angle = targetPoint.getAngle();
        return autoCollectGlobal;
    }

    private final Command autoCollectGlobal = new LambdaCommand(
            () -> runSet = false,
            () -> {

                if (!runSet){
                    preCollect.execute();
                    fourBarState = fourBar.preCollect;

                    queueCommand(extendoTargetPoint);
                    queueCommand(collect);

                    runSet = true;
                }

            },
            () -> fourBarState == fourBar.preCollect && runSet
    );

    public Command spikeTransfer = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
                transferToFar = false;

                fourBarTargetState = fourBar.collect;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 9) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = 90;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab && (griperRotate.getPositionDegrees() < 45 || griperRotate.getPositionDegrees() > 225)){

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*0.4, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*0.4);
                    fourBarTargetState = fourBar.collect;

                    fourBarMainPivot.setPosition(mainPivotPreCollect+30);
                    fourBarSecondPivot.setPosition(secondPivotPreCollect);

                    griperRotate.setPosition(rotateTransfer);
                    turret.setPosition(turretTransferPosition);

                    double oldX = targetPositionManuel.getX();
                    targetPositionManuel = new Vector2D(oldX, 20);

                    keepTargeting = false;

                }else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {
                    double turretTime = Math.abs( turret.getPositionDegrees()-turretTransferPosition)*0.4;


                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam + turretTime)*2.2);
//                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam + turretTime)*spikeTime);
                    fourBarTargetState = fourBar.transferUp;

                    keepTargeting = false;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        setSlideTarget(0);
                        targetPositionManuel = new Vector2D(20, 20);

                        if (horizontalMotor.getCurrentPosition() < 320){
                            if (spikeDriving){
                                TransferAutoSpikeDriving.execute();
                            }else{
                                TransferAutoSpike.execute();
                            }
                        }else{
                            fourBarMainPivot.setPosition(mainPivotPreCollect+15);
                            fourBarSecondPivot.setPosition(secondPivotPreCollect - 40);
                            transferToFar = true;
                        }

                    }

                }

                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
                    if (spikeDriving){
                        TransferAutoSpikeDriving.execute();
                    }else{
                        TransferAutoSpike.execute();
                    }
                    transferToFar = false;
                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> (fourBarState == fourBar.transferUp && slidesReset.isPressed()) || cancelTransfer
    );

    public Command subTransfer = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
                transferToFar = false;

                fourBarTargetState = fourBar.collect;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 9) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = 150;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();

                }else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    double turretTime = Math.abs( turret.getPositionDegrees()-turretTransferPosition)*0.4;

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam + turretTime)*2.2);
                    fourBarTargetState = fourBar.transferUp;

                    keepTargeting = false;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        targetPositionManuel = new Vector2D(20, 20);

                        secondPivotWaitTimer.reset();
                        runSlideSet = true;
                        if (getSlidePositionCM() > 35){
                            slidesWaitTime = 0;
                        }else {
                            slidesWaitTime = (69 - getSlidePositionCM()) * 2.3;
                        }

                        if (spikeDriving){
                            TransferAutoSpikeDriving.execute();
                        }else{
                            TransferAutoSpike.execute();
                        }

//                        if (horizontalMotor.getCurrentPosition() < 320){
//
//                        }else{
//                            fourBarMainPivot.setPosition(mainPivotPreCollect+15);
//                            fourBarSecondPivot.setPosition(secondPivotPreCollect - 40);
//                            transferToFar = true;
//                        }
                    }
                }

                if (secondPivotWaitTimer.milliseconds() > slidesWaitTime && runSlideSet){
                    setSlideTarget(0);
                    runSlideSet = false;
                }

//                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
//                    if (spikeDriving){
//                        TransferAutoSpikeDriving.execute();
//                    }else{
//                        TransferAutoSpike.execute();
//                    }
//                    transferToFar = false;
//                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

                if(isCancelTransferActive() && !breakBeam.isPressed() && clawsState == clawState.grab && abortTimer.milliseconds() > 200 && abortTimer.milliseconds() < 400){
                    preCollect.execute();
                    setClawsState(clawState.drop);
                    fourBarState = fourBar.preCollect;
                    cancelTransfer = true;
                    transferCanceled = true;
                    clearQueue();
                }

            },
            () -> (fourBarState == fourBar.transferUp && slidesReset.isPressed()) || cancelTransfer
    );

    public Command teleopTransfer = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
                transferToFar = false;

                fourBarTargetState = fourBar.collect;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 9) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = 150;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    double turretTime = Math.abs( turret.getPositionDegrees()-turretTransferPosition)*0.4;

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam + turretTime)*2.2);
                    fourBarTargetState = fourBar.transferUp;

                    keepTargeting = false;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        setSlideTarget(0);
                        targetPositionManuel = new Vector2D(20, 20);

                        if (horizontalMotor.getCurrentPosition() < 320){
                            if (spikeDriving){
                                TransferAutoSpikeDriving.execute();
                            }else{
                                TransferAutoSpike.execute();
                            }
                        }else{
                            fourBarMainPivot.setPosition(mainPivotPreCollect+15);
                            fourBarSecondPivot.setPosition(secondPivotPreCollect - 40);
                            transferToFar = true;
                        }

                    }

                }

                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
                    if (spikeDriving){
                        TransferAutoSpikeDriving.execute();
                    }else{
                        TransferAutoSpike.execute();
                    }
                    transferToFar = false;
                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

                if(isCancelTransferActive() && !breakBeam.isPressed() && clawsState == clawState.grab && abortTimer.milliseconds() > 200 && abortTimer.milliseconds() < 400){
                    preCollect.execute();
                    setClawsState(clawState.drop);
                    fourBarState = fourBar.preCollect;
                    cancelTransfer = true;
                    transferCanceled = true;
                    clearQueue();
                }

            },
            () -> (fourBarState == fourBar.transferUp && slidesReset.isPressed()) || cancelTransfer
    );

    public Command chamberCollectSample = new LambdaCommand(
            () -> cancelTransfer = false,
            () -> {

                if (fourBarState == fourBar.collect && clawsState == clawState.drop){

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    clawsState = clawState.grab;

                }else if (fourBarState == fourBar.collect){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowedChamber;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotChamberStowed)*(microRoboticTime+2), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotChamberStowed)*2);

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {
                        fourBarMainPivot.setPosition(126);
                        fourBarSecondPivot.setPosition(275);
                        turret.setPosition(turretTransferPosition);
                        griperRotate.setPosition(rotateTransfer);
                        setSlideTarget(0);
                        targetPositionManuel = new Vector2D(20, 20);
                    }

                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

                if(isCancelTransferActive() && !breakBeam.isPressed() && clawsState == clawState.grab && fourBarTargetState != fourBar.collect && transferCounter < 6){
                    preCollect.execute();
                    setClawsState(clawState.drop);
                    fourBarState = fourBar.preCollect;
                    cancelTransfer = true;
                    transferCanceled = true;
                    clearQueue();
                }

            },
            () -> fourBarState == fourBar.stowedChamber && slideTarget == 0 || cancelTransfer
    );

    public Command stow = new LambdaCommand(
            () -> clawsState = clawState.drop,
            () -> {

                if (fourBarState != fourBar.transferringStates){
                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotStow)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    clawsState = clawState.drop;
                    Stowed.execute();
                    turret.setPosition(turretTransferPosition);
                    griperRotate.setPosition(rotateTransfer);
                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> fourBarState == fourBar.stowed
    );

    public final Command transfer = new LambdaCommand(
            () -> {},
            () -> {
                switch (transferType){
                    case spike:
                        queueCommand(spikeTransfer);

                        queueCommand(delivery.closeGripperSpike);

                        queueCommand(openGripper);
                        break;
                    case teleop:
                        queueCommand(teleopTransfer);

                        queueCommand(delivery.closeGripperSpike);

                        queueCommand(openGripperRetryTeleHigh);
                        break;
                    case subCycles:
                        queueCommand(subTransfer);

                        queueCommand(delivery.closeGripperSpike);

                        queueCommand(openGripper);
                        break;
                    case underChamberCycle:
                        if (fourBarState == fourBar.collect){
                            queueCommand(chamberCollectSample);
                        }else if (fourBarState == fourBar.stowedChamber){
                            queueCommand(teleopTransfer);

                            queueCommand(delivery.closeGripperSpike);

                            queueCommand(openGripperRetryTeleHigh);
                        }
                        break;
                    default:
                }

                if (resetTransfer){
                    resetTransfer = false;
                    transferType = transferTypeSaved;
                }
            },
            () -> true
    );

    public fourBar getFourBarState() {
        return fourBarState;
    }

    public void disableServos(){
        fourBarMainPivot.disableServo();
        fourBarSecondPivot.disableServo();
        griperRotate.disableServo();
        gripServo.disableServo();
        turret.disableServo();
    }

    public static Vector2D rotatePosition(double heading, Vector2D point){
        double Y = (point.getY()) * Math.sin(Math.toRadians(heading)) + (point.getX()) * Math.cos(Math.toRadians(heading));
        double X = (point.getY()) * Math.cos(Math.toRadians(heading)) - (point.getX()) * Math.sin(Math.toRadians(heading));

        return new Vector2D(X, Y);
    }

    public static Vector2D rotatePositionToGlobal(double heading, Vector2D point){
        double X = (point.getX()) * Math.cos(Math.toRadians(heading)) - (point.getY()) * Math.sin(Math.toRadians(heading));
        double Y = (point.getX()) * Math.sin(Math.toRadians(heading)) + (point.getY()) * Math.cos(Math.toRadians(heading));

        return new Vector2D(X, Y);
    }

    public double getSlidePositionCM(){
        double ticksPerCM = (double) 18 / 190;
        return horizontalMotor.getCurrentPosition() * ticksPerCM;
    }

    public void setSlideTarget(double slideTarget) {

        if (this.slideTarget == 0 && slideTarget > 0 && resettingSlides){
            resettingSlides = false;
        }

        if (slideTarget < 0){
            this.slideTarget = 0;
        }else {
            if (Math.abs(slideTarget - getSlidePositionCM()) > 20){
                longTarget = true;
            }else {
                longTarget = false;
            }
            this.slideTarget = slideTarget;
        }

        if (slideTarget > 69){
            this.slideTarget = 69;
        }
    }

    public void armEndPointIncrement(double incrementHorizontal, double incrementVertical, boolean globalTargeting){
        double oldX = targetPositionManuel.getX();
        double oldY = targetPositionManuel.getY();

        if (oldY+incrementHorizontal > (clawOffsetFromSlides*2)){
            incrementHorizontal = 0;
            oldY = clawOffsetFromSlides*2;
        } else if (oldY+incrementHorizontal < 0) {
            oldY = 0;
            incrementHorizontal = 0;
        }

        double finalXTarget = oldX+incrementVertical;

        if (finalXTarget > 70){
            finalXTarget = 70;
        }

        targetPositionManuel = new Vector2D(finalXTarget, oldY+incrementHorizontal);

        if (globalTargeting){
            setSlideTarget(calculateKinematicsGlobal());
        }else{
            keepTargeting = false;

            if (getCurrentCommand() == extendoTargetPoint){
                overrideCurrent(true, defaultCommand);
            }

            setSlideTarget(calculateKinematics());
        }

    }

    private double calculateKinematics(){

        double yError = clawOffsetFromSlides - targetPositionManuel.getY();

        double angle = Math.toDegrees(Math.acos(yError / clawOffsetFromSlides));

        double hypotSquared = (clawOffsetFromSlides * clawOffsetFromSlides) - (Math.abs(yError) * Math.abs(yError));

        double slideOffset = Math.sqrt(hypotSquared);

        turret.setPosition(77.5 + angle);

        turretTargetPosition = 77.5 + angle;

        if (targetPositionManuel.getY() < clawOffsetFromSlides){
            parallelAngle = 90 + (turretTargetPosition - turretTransferPosition);
        }else {
            parallelAngle = 90 + (turretTargetPosition - turretTransferPosition);
        }

        double perAngle = 0;

        if (parallelAngle > 90){
            perAngle = parallelAngle - manualAngle;
        }else if (parallelAngle < 90){
            perAngle = parallelAngle + manualAngle;
        }

        if (fourBarState == fourBar.preCollect){
            griperRotate.setPosition(perAngle);
        }

        double turretOffset = 5;

        return targetPositionManuel.getX() - slideOffset + turretOffset;
    }

    private double calculateKinematicsGlobal()  {

        Vector2D errors = rotatePosition(RobotPosition.getPivot(), new Vector2D(RobotPosition.getHorizontal() - targetPosition.getY(), targetPosition.getX() - RobotPosition.getVertical()));

//        System.out.println("RobotPosition.getPivot(): " + RobotPosition.getPivot());
//
//        System.out.println("errors.gety(): " + targetPosition.getY());
//        System.out.println("errors.getx(): " + (targetPosition.getX()));
//
//        System.out.println("errors.getY(): " + errors.getY());
//        System.out.println("errors.getX(): " + errors.getX());

        targetPositionManuel = new Vector2D(errors.getX() - robotLength, clawOffsetFromSlides + errors.getY());

        double slideOffset = 0;

        if (errors.getY() < -20 || errors.getY() > 20){
            return 18763;
        }else{

            double angle = Math.toDegrees((Math.acos(errors.getY()) / clawOffsetFromSlides));

            double realAngle;

            double perAngle = 0;

            double turretPosition = 77.5 + angle;

            parallelAngle = 90 + (turretTargetPosition - turretTransferPosition);

//            if (errors.getY() < clawOffsetFromSlides){
//                parallelAngle = 90 + (turretTargetPosition - turretTransferPosition);
//            }else {
//                parallelAngle = 90 + (turretTargetPosition - turretTransferPosition);
//            }

            double hypotSquared = (clawOffsetFromSlides * clawOffsetFromSlides) - (Math.abs(errors.getY()) * Math.abs(errors.getY()));

            slideOffset = Math.sqrt(hypotSquared);

            double turretOffset = 5;

            double returnTarget = ((((errors.getX()) - robotLength) - slideOffset) + turretOffset);

            if (returnTarget < 65){

                targetPositionManuel = new Vector2D(errors.getX() - robotLength, clawOffsetFromSlides - errors.getY());

                turret.setPosition(turretPosition);

                turretTargetPosition = turretPosition;

                if(angleRecheck){
                    if (parallelAngle > 90){
                        goPositive= true;
                    }else{
                        goPositive= false;
                    }
                }

                if (goPositive){
                    perAngle = parallelAngle - 90;
//                    System.out.println("perAngle positive: " + perAngle);
                }else{
                    perAngle = parallelAngle + 90;
//                    System.out.println("perAngle negitive: " + perAngle);
                }

                realAngle = perAngle - this.angle;

                if(angleRecheck){
                    if (realAngle >= 180){
                        goNegative = true;
                        over = true;
                    } else if (realAngle <= 0) {
                        goNegative = true;
                        over = false;
                    }else {
                        goNegative = false;
                    }

                    angleRecheck = false;
                }

                if (goNegative){
                    if (over){
                        realAngle = realAngle - 180;
                    } else {
                        realAngle = realAngle + 180;
                    }
                }

                griperRotate.setPosition(realAngle);

//                manualAngle = realAngle;
//                System.out.println("Turret target: " + turretPosition);
//                System.out.println("Slide target: " + returnTarget);

                return returnTarget;

            }else {
                return 18763;
            }

        }

    }

    public void runReset(){
        slideTarget = 0;
        extendoPower = -0.55;
        resettingSlides = true;
    }

    boolean cancelTransferActive = true;

    public boolean isCancelTransferActive() {
        return cancelTransferActive;
    }

    public void setCancelTransfer(boolean cancelTransfer) {
        this.cancelTransferActive = cancelTransfer;
    }

    public clawState getClawsState() {
        return clawsState;
    }

    public void setClawsState(clawState clawsState) {
        this.clawsState = clawsState;
    }

    public boolean isTransferCanceled() {
        return transferCanceled;
    }

    public void resetTransferCanceled() {
        transferCanceled = false;
    }

    public void setBraking(boolean braking) {
        this.braking = braking;
        brakingTimer.reset();
    }

    public tranfer getTransferType() {
        return transferType;
    }

    public void setTransferType(tranfer transferType) {
        this.transferType = transferType;
    }

    public double getSlideTarget() {
        return slideTarget;
    }

    public void updateRobotPosition(RobotPower robotPosition){
        RobotPosition = robotPosition;
    }

    public void stopTargeting() {
        this.keepTargeting = false;
    }

    public void setSpikeDriving(boolean spikeDriving) {
        this.spikeDriving = spikeDriving;
    }

    public void setTargeting(targetingTypes Targeting) {
        this.targeting = Targeting;
    }

    public boolean isTransferRetryBoolean() {
        return transferRetryBoolean;
    }

    public void setTransferRetryBoolean(boolean transferRetryBoolean) {
        this.transferRetryBoolean = transferRetryBoolean;
    }

}
