package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Point;

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

    //abort collection variables
    ElapsedTime commandTimer = new ElapsedTime();
    ElapsedTime abortTimer = new ElapsedTime();
    double abortTime = 0;
    boolean transferToFar = false;

    // slides
    public MotorEx horizontalMotor = new MotorEx();
    double extendoPower = 0;

    //servos
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoDegrees gripServo = new ServoDegrees();
    public ServoDegrees turret =new ServoDegrees();

    // sensors
    public AxonEncoder turretPosition = new AxonEncoder();
    public TouchSensor breakBeam;
    public TouchSensor slidesReset;

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 600 / 360;
    double microRoboticTime = (double) 500 / 360;
    double gripperOpenTime = 200;

    //robot and arm size
    double robotLength = 35 * 0.5;
    double clawOffsetFromSlides = 20;

    boolean runSet = false;
    boolean TransferDrop = false;
    ElapsedTime WaitForTranferDrop = new ElapsedTime();

    /**states*/
    public enum fourBar{
        preCollect,
        clip,
        collect,
        detectingSample,
        transferringStates,
        preClipLow,
        stowedChamber,
        transferUp,
        stowed,
        wallCollect,
        wallRotate,
        transferInt
    }

    public enum clawState{
        drop,
        grab,
        slightRelease,
        griperHang,
        openFull
    }

    public enum tranfer{
        auto,
        spike,
        sample,
        normalSlam,
        slowBackup,
        chamberCollect,
        preClip,
        specimen
    }

    /**
     * collect position values
     * */
    double mainPivotCollect = 86;
    double secondPivotCollect = 317;

    /**
     * collect position values
     * */
    double mainPivotCollectClip = 88;
    double secondPivotCollectClip = 300;

    /**
     * wall collect position values
     * */
    double mainPivotWallCollect = 165;
    double secondPivotWallCollect = 230;


    /**
     * preCollect position values
     * */
    double mainPivotPreCollect = 115;
    double secondPivotPreCollect = 305;

    /**
     * preCollect position values
     * */
    double mainPivotMidTransfer = 180;
    double secondPivotMidTransfer = 160;

    /**
     * preCollect position values
     * */
    double mainPivotRetryTransfer = 170;
    double secondPivotMidRetryTransfer = 160;

    /**
     * low chamber position values
     * */
    double mainPivotLowChamberPreClip = 92;
    double secondPivotLowChamberPreClip = 220;

    /**
     * low chamber position values
     * */
    double mainPivotLowChamberClip = 92;
    double secondPivotLowChamberClip = 220;

    /**
     * stowed position values
     * */
    double mainPivotStow = 165;
    double secondPivotStow = 152;
    public double turretTransferPosition = 167.5;

    /**
     * stowed position values
     * */
    double mainPivotSampleStow = 170;
    double secondPivotSampleStow = 140;

    /**
     * stowed position values
     * */
    double mainPivotChamberStowed = 125;
    double secondPivotChamberStowed = 285;

    /**
     * stowed position values
     * */
    double mainPivotTransferSlam = 180;
    double secondPivotTransferSlam = 144;

    /**
     * stowed position values
     * */
    double mainPivotSampleTransfer = 182;
    double secondPivotSampleTransfer = 128;

    /**
     * stowed position values
     * */
    double mainPivotTransferAuto = 180;
    double secondPivotTransferAuto = 142;

    /**
     * stowed position values
     * */
    double mainPivotTransfer = 198;
    double secondPivotTransfer = 148;
    double rotateTransfer = 180;

    /**
     * stow position values
     * */
    double mainPivotChamberCollect = 140;
    double secondPivotChamberCollect = 310;

    public double mainPivotHang = 83;
    public double secondPivotHang = 210;

    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;

    boolean autoCollecting = false;

    /**enum states*/
    private fourBar fourBarState = fourBar.stowed;
    private fourBar fourBarTargetState = fourBar.stowed;
    private clawState clawsState = clawState.drop;
    private tranfer transferType = tranfer.normalSlam;

    /**PID controllers**/
    PIDController adjustment = new PIDController(0.015, 0, 0.05);
    PIDController adjustmentClose = new PIDController(0.01, 0, 0.009);

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
    double gripperDrop = 132;
    double gripperGrab = 78;
    double gripperHangGrab = 120;
    double gripperSlightRelease = 86;
    double gripperOpenFull = 132;

    boolean braking = false;
    ElapsedTime brakingTimer = new ElapsedTime();

    public boolean resettingSlides = false;

    RobotPower RobotPosition = new RobotPower();

    public Point targetPointGlobal;
    public double angle;
    public double parallelAngle;
    public double manualAngle = 0;
    int detectionAttemptsCounter = 0;

    public Collection(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }

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

        turretPosition.setOffset(-10.5);

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(335);
        turret.setRange(335);

        turret.setOffset(-4);
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
        griperRotate.setOffset(10);
        griperRotate.setPosition(180);

        setClawsState(clawState.drop);
        gripServo.setPosition(90);

        Stowed.execute();
        runReset();
    }


    public void setHangHold(boolean hangHold) {
        this.hangHold = hangHold;
    }

    @Override
    public void execute() {

        executeEX();

        double ticksPerCM = (double) 205 / 18;
        double error;

        if (hangHold) {
            horizontalMotor.update(0);

        } else {
            if ((slideTarget*ticksPerCM) < 570){
                error = Math.abs((slideTarget*ticksPerCM) - (double) horizontalMotor.getCurrentPosition());
            }else {
                error = Math.abs((570) - (double) horizontalMotor.getCurrentPosition());
            }


            if (!resettingSlides && !slidesReset.isPressed() && slideTarget == 0 && Math.abs(horizontalMotor.getVelocity()) < 10 && horizontalMotor.getCurrentPosition() < 100){
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

                if (longTarget){
                    extendoPower = Range.clip(adjustment.calculate((slideTarget * ticksPerCM), horizontalMotor.getCurrentPosition()), -1, 1);
                }else{
                    extendoPower = Range.clip(adjustmentClose.calculate((slideTarget * ticksPerCM), horizontalMotor.getCurrentPosition()), -1, 1);
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
                gripServo.setPosition(gripperDrop);
            } else if (clawsState == clawState.slightRelease){
                gripServo.setPosition(gripperSlightRelease);
            }else if (clawsState == clawState.openFull){
                gripServo.setPosition(gripperOpenFull);
            }else if (clawsState == clawState.griperHang){
                gripServo.setPosition(gripperHangGrab);
            }

            if (isTransferCanceled() && fourBarState != fourBar.preCollect){
                transferCanceled = false;
            }

            horizontalMotor.update(Range.clip(extendoPower, -1, 0.8));
        }
    }

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    private final Command WallCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotWallCollect);
                fourBarSecondPivot.setPosition(secondPivotWallCollect);
                griperRotate.setPosition(0);
            }
    );

    private final Command PreClip = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotLowChamberPreClip);
                fourBarSecondPivot.setPosition(secondPivotLowChamberPreClip);
                griperRotate.setPosition(0);
            }
    );

    private final Command Clip = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotLowChamberClip);
                fourBarSecondPivot.setPosition(secondPivotLowChamberClip);
                griperRotate.setPosition(0);
            }
    );

    private final Command preCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPreCollect);
                fourBarSecondPivot.setPosition(secondPivotPreCollect);
            }
    );

    private final Command hang = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotHang);
                fourBarSecondPivot.setPosition(secondPivotHang);
            }
    );

    private final Command midTransfer = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotMidTransfer);
                fourBarSecondPivot.setPosition(secondPivotMidTransfer);
            }
    );

    private final Command transferRetry = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotRetryTransfer);
                fourBarSecondPivot.setPosition(secondPivotMidRetryTransfer);
            }
    );

    private final Command Collect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotCollect);
                fourBarSecondPivot.setPosition(secondPivotCollect);
            }
    );

    private final Command CollectCLip = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotCollectClip);
                fourBarSecondPivot.setPosition(secondPivotCollectClip);
            }
    );

    private final Command Transfer = new Execute(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotTransfer);
                fourBarMainPivot.setPosition(mainPivotTransfer);

                clawsState = clawState.grab;
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

    private final Command TransferSample = new Execute(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotSampleTransfer);
                fourBarMainPivot.setPosition(mainPivotSampleTransfer);

                clawsState = clawState.grab;
            }
    );

    private final Command SampleStowed = new Execute(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotSampleStow);
                fourBarMainPivot.setPosition(mainPivotSampleStow);

                clawsState = clawState.grab;
            }
    );

    public final Command ChamberStowed = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotChamberStowed);
                fourBarSecondPivot.setPosition(secondPivotChamberStowed);
                turret.setPosition(turretTransferPosition);
            }
    );

    public final Command ChamberCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotChamberCollect);
                fourBarSecondPivot.setPosition(secondPivotChamberCollect);
            }
    );

    public final Command Stowed = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotStow);
                fourBarSecondPivot.setPosition(secondPivotStow);
            }
    );

    public Command transfer(tranfer transferType){
        this.transferType = transferType;
        return transfer;
    }

    public Command transfer(tranfer transferType, boolean queueArmDown){
        this.transferType = transferType;
        queueCommand(collect);
        return transfer;
    }

    public final Command transferDrop = new LambdaCommand(
            () -> {
                Transfer.execute();
                setClawsState(clawState.slightRelease);
                WaitForTranferDrop.reset();

                TransferDrop = false;
                fourBarState = fourBar.stowed;
            },
            () -> {
                setClawsState(clawState.slightRelease);
                if (WaitForTranferDrop.milliseconds() > 400){
                    TransferDrop = true;
                }
            },
            () -> TransferDrop

    );

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
                TransferAuto.execute();
                setClawsState(clawState.slightRelease);
                WaitForTranferDrop.reset();

                TransferDrop = false;
                fourBarState = fourBar.stowed;
            },
            () -> {
                setClawsState(clawState.slightRelease);
                if (WaitForTranferDrop.milliseconds() > 40){
                    TransferDrop = true;
                }
            },
            () -> TransferDrop

    );

    public final Command retryTransfer = new LambdaCommand(
            () -> {
                transferRetry.execute();
                setClawsState(clawState.grab);
                WaitForTranferDrop.reset();

                TransferDrop = false;
                fourBarState = fourBar.stowed;
            },
            () -> {
                if (WaitForTranferDrop.milliseconds() > 600){
                    TransferDrop = true;
                } else if (WaitForTranferDrop.milliseconds() > 300) {
                    TransferAuto.execute();
                }
            },
            () -> TransferDrop

    );

    public final Command transferDropSampleTeleop = new LambdaCommand(
            () -> {
                TransferSample.execute();
                setClawsState(clawState.slightRelease);
                WaitForTranferDrop.reset();

                TransferDrop = false;
                fourBarState = fourBar.stowed;
            },
            () -> {
                setClawsState(clawState.slightRelease);
                if (WaitForTranferDrop.milliseconds() > 150){
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
            () -> fourBarTimer.milliseconds() > 80
    );

    public final Command openGripperSample = new LambdaCommand(
            () -> {
                setClawsState(clawState.drop);
                SampleStowed.execute();
                fourBarTimer.reset();
            },
            () -> {
            },
            () -> fourBarTimer.milliseconds() > 80
    );

    public Command wallCollect = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState != fourBar.transferringStates && fourBarState != fourBar.wallCollect){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.wallCollect;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotWallCollect)*4, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotWallCollect)*6);

                    WallCollect.execute();

                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> fourBarState == fourBar.wallCollect
    );

    public Command collect = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCollect)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotCollect)*1);

                    if (getTransferType() == tranfer.preClip){
                        CollectCLip.execute();
                    }else {
                        Collect.execute();
                    }

                } else if (fourBarState == fourBar.collect) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotPreCollect)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*3);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;

                    preCollect.execute();

                }else if (!(fourBarState == fourBar.transferringStates)){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotPreCollect)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*3);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;

                    preCollect.execute();
                    griperRotate.setPosition(180);

                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> !(fourBarState == fourBar.transferringStates) && fourBarTimer.milliseconds() > transferWaitTime
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
                    gripServo.setPosition(gripperGrab);

                } else if (fourBarState == fourBar.transferInt) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.abs(turret.getPositionDegrees()- 46)*4;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    turret.setPosition(46);

                }else if (fourBarState == fourBar.transferUp){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotHang)*3, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotHang)*3);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowedChamber;

                    hang.execute();

                }else if (fourBarState == fourBar.stowedChamber) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.abs(turret.getPositionDegrees()- 56)*4;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    turret.setPosition(56);

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

    public Command preCollectNoRotate(double rotateTarget){
        griperRotate.setPosition(rotateTarget);
        return preCollectNoRotate;
    }

    public Command preCollectNoRotate = new LambdaCommand(
            () -> {},
            () -> {

                  if (!(fourBarState == fourBar.transferringStates)){

                        fourBarTimer.reset();
                        transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotPreCollect)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*microRoboticTime);
                        fourBarState = fourBar.transferringStates;
                        fourBarTargetState = fourBar.preCollect;

                        setClawsState(clawState.openFull);
                        preCollect.execute();
                  }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> !(fourBarState == fourBar.transferringStates) && fourBarTimer.milliseconds() > transferWaitTime && fourBarState != fourBar.transferInt
    );

    public Command extendoTargetPoint(Vector2D targetPoint){
        targetPosition = targetPoint;
        return extendoTargetPoint;
    }

    public Command extendoTargetPoint = new LambdaCommand(
            () -> {
                abortTimer.reset();

                runSet = false;
                abortTime = 1000;
            },
            () -> {

//                if (!runSet){
                double newSlideTarget = calculateKinematicsGlobal();
//                System.out.println("SLIDE TARGET" + newSlideTarget);

                if (newSlideTarget != 18763){
                    setSlideTarget(newSlideTarget);
//                    System.out.println("RAN SET IN EXTENDO TARGETING" + runSet);
                    runSet = true;
                }

//                System.out.println("Condition 1" + (runSet && Math.abs(getSlideTarget() - getSlidePositionCM()) < 1.1 && Math.abs(horizontalMotor.getVelocity()) < 10 && Math.abs(extendoPower) < 0.1 && getSlideTarget() > 1));
//                System.out.println("Condition 2" + (!runSet && abortTimer.milliseconds() > abortTime));

//                }
            },
            () -> runSet && Math.abs(getSlideTarget() - getSlidePositionCM()) < 1 && Math.abs(horizontalMotor.getVelocity()) < 20 && Math.abs(extendoPower) < 0.1 && getSlideTarget() > 1 && Math.abs(turretTargetPosition - turretPosition.getPosition()) < 6 || !runSet && abortTimer.milliseconds() > abortTime
    );

    public Command autoCollectGlobal(TargetSample targetPoint){
//        System.out.println("Running the global collect" + targetPoint.getTargetPoint().getX() + " : " + targetPoint.getTargetPoint().getY());
        targetPosition = targetPoint.getTargetPoint();
        angle = targetPoint.getAngle();
        return autoCollectGlobal;
    }

    private final Command autoCollectGlobal = new LambdaCommand(
            () -> runSet = false,
            () -> {

                if (!runSet){
                    if (fourBarMainPivot.getPositionDegrees() > 150){
                        preCollect.execute();
                        fourBarState = fourBar.preCollect;
                    }

                    queueCommand(extendoTargetPoint);
                    queueCommand(collect);

                    runSet = true;
                }

            },
            () -> fourBarState == fourBar.preCollect && runSet
    );

    public void targetPointWithExtendo(Vector2D targetPoint){

        targetPosition = targetPoint;

        setSlideTarget(calculateKinematicsGlobal());

    }

    public void targetPointWithExtendoNoArm(Vector2D targetPoint){

        targetPosition = targetPoint;

        calculateKinematicsGlobal();
    }

    public Command transferSlowBackup = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 5) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*6, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransfer)*4);
                    fourBarTargetState = fourBar.transferUp;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {
                        setSlideTarget(0);
                        targetPositionManuel = new Vector2D(20, 20);
                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        midTransfer.execute();
                    }
                }

                if (clawsState == clawState.grab && fourBarTargetState != fourBar.collect){
                    transferCounter++;
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

    public Command transferSlam = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
                transferToFar = false;
                runningCheck = false;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 5) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = 200;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();
                    runningCheck = true;

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab && (griperRotate.getPositionDegrees() < 90)){

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*3, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*microRoboticTime);
                    fourBarTargetState = fourBar.collect;

                    fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                    fourBarSecondPivot.setPosition(secondPivotPreCollect);

                    griperRotate.setPosition(rotateTransfer);
                    turret.setPosition(turretTransferPosition);

                    double oldX = targetPositionManuel.getX();
                    targetPositionManuel = new Vector2D(oldX, 20);

                }else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*3, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam)*4);
                    fourBarTargetState = fourBar.transferUp;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){
                        double oldX = targetPositionManuel.getX();
                        targetPositionManuel = new Vector2D(oldX, 20);
                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        setSlideTarget(0);
                        targetPositionManuel = new Vector2D(20, 20);

                        if (horizontalMotor.getCurrentPosition() < 320){
                            TransferSlam.execute();
                        }else{
                            fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                            fourBarSecondPivot.setPosition(secondPivotPreCollect - 60);
                            transferToFar = true;
                        }

                    }

                }

                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
                    TransferSlam.execute();
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

    public Command transferAuto = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
                transferToFar = false;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 5) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab && (griperRotate.getPositionDegrees() < 100)){

//                     || (turret.getPositionDegrees() - turretTransferPosition) > 20)  || (turret.getPositionDegrees() - turretTransferPosition) < -20
                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(turret.getPositionDegrees()-turretTransferPosition)*3);
                    fourBarTargetState = fourBar.collect;

                    fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                    fourBarSecondPivot.setPosition(secondPivotPreCollect);

                    griperRotate.setPosition(rotateTransfer);
                    turret.setPosition(turretTransferPosition);

                }else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1.5, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam)*4);
                    fourBarTargetState = fourBar.transferUp;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){
                        double oldX = targetPositionManuel.getX();
                        targetPositionManuel = new Vector2D(oldX, 20);
                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        setSlideTarget(0);

                        if (horizontalMotor.getCurrentPosition() < 320){

                            targetPositionManuel = new Vector2D(20, 20);

                            TransferAuto.execute();
                        }else{
                            fourBarMainPivot.setPosition(mainPivotPreCollect+5);
                            fourBarSecondPivot.setPosition(secondPivotPreCollect - 40);
                            transferToFar = true;
                        }

                    }

                }

                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
                    TransferAuto.execute();
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

//                System.out.println("counter: " + transferCounter);
//                System.out.println("fourBarMainPivot.getPositionDegrees(): " + fourBarMainPivot.getPositionDegrees());
//                System.out.println("cancel transfer: " + cancelTransfer);

            },
            () -> (fourBarState == fourBar.transferUp && slidesReset.isPressed()) || cancelTransfer
    );

    public Command transferSpec = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
                transferToFar = false;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 5) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab && (griperRotate.getPositionDegrees() < 100)){

//                     || (turret.getPositionDegrees() - turretTransferPosition) > 20)  || (turret.getPositionDegrees() - turretTransferPosition) < -20
                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(turret.getPositionDegrees()-turretTransferPosition)*3);
                    fourBarTargetState = fourBar.collect;

                    fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                    fourBarSecondPivot.setPosition(secondPivotPreCollect);

                    griperRotate.setPosition(rotateTransfer);
                    turret.setPosition(turretTransferPosition);

                }else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1.5, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam)*4);
                    fourBarTargetState = fourBar.transferUp;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){
                        double oldX = targetPositionManuel.getX();
                        targetPositionManuel = new Vector2D(oldX, 20);
                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        setSlideTarget(0);
                        targetPositionManuel = new Vector2D(20, 20);

                        if (horizontalMotor.getCurrentPosition() < 320){
                            TransferAuto.execute();
                        }else{
                            fourBarMainPivot.setPosition(mainPivotPreCollect + 5);
                            fourBarSecondPivot.setPosition(secondPivotPreCollect - 40);
                            transferToFar = true;
                        }

                    }

                }

                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
                    TransferAuto.execute();
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

//                System.out.println("counter: " + transferCounter);
//                System.out.println("fourBarMainPivot.getPositionDegrees(): " + fourBarMainPivot.getPositionDegrees());
//                System.out.println("cancel transfer: " + cancelTransfer);

            },
            () -> (fourBarState == fourBar.transferUp && slidesReset.isPressed()) || cancelTransfer
    );

    public Command transferSpike = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
                transferToFar = false;

                fourBarTargetState = fourBar.collect;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 5) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = 120;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab && (griperRotate.getPositionDegrees() < 100)){

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*1);
                    fourBarTargetState = fourBar.collect;

                    fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                    fourBarSecondPivot.setPosition(secondPivotPreCollect);

                    griperRotate.setPosition(rotateTransfer);
                    turret.setPosition(turretTransferPosition);

                    double oldX = targetPositionManuel.getX();
                    targetPositionManuel = new Vector2D(oldX, 20);

                }else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1.5, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam)*3.5);
                    fourBarTargetState = fourBar.transferUp;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        setSlideTarget(0);
                        targetPositionManuel = new Vector2D(20, 20);

                        if (horizontalMotor.getCurrentPosition() < 320){
                            TransferAuto.execute();
                        }else{
                            fourBarMainPivot.setPosition(mainPivotPreCollect+5);
                            fourBarSecondPivot.setPosition(secondPivotPreCollect - 40);
                            transferToFar = true;
                        }

                    }

                }

                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
                    TransferAuto.execute();
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

    public Command transferSampleTeleop = new LambdaCommand(
            () -> {
                cancelTransfer = false;
                transferCounter = 0;
                transferToFar = false;
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 5) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    abortTimer.reset();

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab){

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(fourBarSecondPivot.getPositionDegrees()-(secondPivotPreCollect - 120))*2);
                    fourBarTargetState = fourBar.transferInt;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {
                        fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                        fourBarSecondPivot.setPosition(secondPivotPreCollect-120);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        double oldX = targetPositionManuel.getX();
                        targetPositionManuel = new Vector2D(oldX, 20);
                    }

                }else if (!cancelTransfer && fourBarState == fourBar.transferInt && clawsState == clawState.grab) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1.5, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotSampleTransfer)*8);
                    fourBarTargetState = fourBar.transferUp;

                    setClawsState(clawState.grab);

                    griperRotate.setPosition(rotateTransfer);
                    turret.setPosition(turretTransferPosition);

                    setSlideTarget(0);

                    targetPositionManuel = new Vector2D(20, 20);

                    TransferSample.execute();

                }

                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
                    TransferSample.execute();
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

    public Command wallTransfer = new LambdaCommand(
            () -> {
                cancelTransfer = false;
            },
            () -> {

                if (fourBarState == fourBar.wallCollect && (clawsState == clawState.drop || clawsState == clawState.openFull)) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.wallCollect;

                    autoCollecting = false;

                }else if (fourBarState == fourBar.wallCollect && clawsState == clawState.grab){

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotWallCollect+6)*(microRoboticTime), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotWallCollect-40)*microRoboticTime);
                    fourBarTargetState = fourBar.transferInt;

                    fourBarMainPivot.setPosition(mainPivotWallCollect+6);
                    fourBarSecondPivot.setPosition(secondPivotWallCollect-40);

                } else if (fourBarState == fourBar.transferInt) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*5, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*2);
                    fourBarTargetState = fourBar.wallRotate;

                    griperRotate.setPosition(rotateTransfer);
                    turret.setPosition(turretTransferPosition);

                    double oldX = targetPositionManuel.getX();
                    targetPositionManuel = new Vector2D(oldX, 20);

                } else if (fourBarState == fourBar.wallRotate) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotTransfer)*(microRoboticTime+10), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransfer)*10);
                    fourBarTargetState = fourBar.transferUp;

                    setSlideTarget(0);
                    targetPositionManuel = new Vector2D(20, 20);

                    if (horizontalMotor.getCurrentPosition() < 300){
                        Transfer.execute();
                        commandTimer.reset();
                    }else{
                        transferToFar = true;
                    }

                }

                if (horizontalMotor.getCurrentPosition() < 300 && transferToFar){
                    Transfer.execute();
                    transferToFar = false;
                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }
            },
            () -> (fourBarState == fourBar.transferUp || cancelTransfer) && slidesReset.isPressed()
    );

    public Command chamberCollect = new LambdaCommand(
            () -> {
                cancelTransfer = false;
            },
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
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotChamberStowed)*(microRoboticTime+10), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotChamberStowed)*10);

                    ChamberStowed.execute();

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {
                        griperRotate.setPosition(90);
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

    public Command clip = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.preClipLow){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotLowChamberClip)*4, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotLowChamberClip)*6);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.clip;

                    Clip.execute();

                }else if (fourBarState == fourBar.clip && clawsState == clawState.grab){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.clip;
                    transferWaitTime = gripperOpenTime;

                    setClawsState(clawState.drop);

                }else if (fourBarState == fourBar.clip && clawsState == clawState.drop){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotPreCollect)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*2);

                    Stowed.execute();

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.stowed
    );

    public Command preClip = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.collect && clawsState == clawState.drop){

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    clawsState = clawState.grab;

                }else if (fourBarState == fourBar.collect && clawsState == clawState.grab){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preClipLow;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotPreCollect)*4, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*6);

                    PreClip.execute();

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.preClipLow
    );

    public Command stow = new LambdaCommand(
            () -> {
                clawsState = clawState.drop;
            },
            () -> {

                if (fourBarState != fourBar.transferringStates){
                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-180)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransfer)*microRoboticTime);
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
                    case auto:
                        queueCommand(transferAuto);

                        queueCommand(transferDropAuto);

                        queueCommand(delivery.closeGripper);

                        queueCommand(openGripper);
                        break;
                    case specimen:
                        queueCommand(transferSpec);

                        queueCommand(transferDropAuto);

                        queueCommand(delivery.closeGripper);

                        queueCommand(openGripper);
                        break;
                    case normalSlam:
                        queueCommand(transferSlam);

                        queueCommand(transferDropSlam);

                        queueCommand(delivery.closeGripper);

                        queueCommand(openGripper);
                        break;
                    case spike:
                        queueCommand(transferSpike);

                        queueCommand(transferDropAuto);

                        queueCommand(delivery.closeGripper);

                        queueCommand(openGripper);
                        break;
                    case sample:
                        queueCommand(delivery.transferSample);

                        queueCommand(transferSampleTeleop);

                        queueCommand(delivery.closeGripperSample);

                        queueCommand(openGripper);
                        break;
                    case slowBackup:
                        queueCommand(transferSlowBackup);

                        queueCommand(transferDrop);

                        queueCommand(delivery.closeGripper);

                        queueCommand(openGripper);
                        break;
                    case chamberCollect:
                        queueCommand(chamberCollect);
                        break;
                    case preClip:
                        queueCommand(preClip);
                        break;
                    default:
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
        double ticksPerCM = (double) 18 / 205;
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

        if (slideTarget > 60){
            this.slideTarget = 60;
        }
    }

    public void armEndPointIncrement(double incrementHorizontal, double incrementVertical, boolean globalTargeting){
        double oldX = targetPositionManuel.getX();
        double oldY = targetPositionManuel.getY();

        if (oldY+incrementHorizontal > clawOffsetFromSlides*2){
            incrementHorizontal = 0;
            oldY = clawOffsetFromSlides*2;
        } else if (oldY+incrementHorizontal < 0) {
            oldY = 0;
            incrementHorizontal = 0;
        }

        targetPositionManuel = new Vector2D(oldX+incrementVertical, oldY+incrementHorizontal);

        if (globalTargeting){
            setSlideTarget(calculateKinematicsGlobal());
        }else{
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

        if (targetPositionManuel.getY() < clawOffsetFromSlides+8){
            parallelAngle = 180 + (turret.getPositionDegrees() - turretTransferPosition);
        }else {
            parallelAngle = 0 + (turret.getPositionDegrees() - turretTransferPosition);
        }

        double perAngle = 0;

        if (parallelAngle > 90){
            perAngle = parallelAngle - manualAngle;
        }else if (parallelAngle < 90){
            perAngle = parallelAngle + manualAngle;
        }

        griperRotate.setPosition(perAngle);

        return targetPositionManuel.getX() - slideOffset;
    }

    private double calculateKinematicsGlobal(){

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

            double angle = Math.toDegrees(Math.acos(errors.getY() / clawOffsetFromSlides));

            if (errors.getY() < -6){
                parallelAngle = -15 + ((77.5 + angle) - turretTransferPosition);
            }else {
                parallelAngle = 165 + ((77.5 + angle) - turretTransferPosition);
            }

            double realAngle;

            if (this.angle > 85 || this.angle < -85){
                realAngle = parallelAngle;
            }else{
                double perAngle = 0;

                if (parallelAngle > 90){
                    perAngle = parallelAngle - 90;
                }else if (parallelAngle < 90){
                    perAngle = parallelAngle + 90;
                }

                realAngle = perAngle - this.angle;

                if (realAngle > 180){
                    realAngle = realAngle - 180;
                } else if (realAngle < 0) {
                    realAngle = realAngle + 180;
                }
            }

            Vector2D positionToAdd = new Vector2D(2, 0);

            if (parallelAngle < 90){
                double offsetFromMiddle = realAngle - parallelAngle;

//                System.out.println("offsetFromMiddle 1: " + offsetFromMiddle);

                positionToAdd = rotatePositionToGlobal(offsetFromMiddle, new Vector2D(2, 0));

                angle = Math.toDegrees(Math.acos((errors.getY()+positionToAdd.getY()) / clawOffsetFromSlides));

            }else{
                double offsetFromMiddle = realAngle - parallelAngle;

//                System.out.println("offsetFromMiddle 2: " + offsetFromMiddle);

                positionToAdd = rotatePositionToGlobal(offsetFromMiddle, new Vector2D(2, 0));

                angle = Math.toDegrees(Math.acos((errors.getY()-positionToAdd.getY()) / clawOffsetFromSlides));
            }

            if (errors.getY() < -6){
                parallelAngle = -15 + ((77.5 + angle) - turretTransferPosition);
            }else {
                parallelAngle = 165 + ((77.5 + angle) - turretTransferPosition);
            }

            if (this.angle > 85 || this.angle < -85){
                realAngle = parallelAngle;
            }else{
                double perAngle = 0;

                if (parallelAngle > 90){
                    perAngle = parallelAngle - 90;
                }else if (parallelAngle < 90){
                    perAngle = parallelAngle + 90;
                }

                realAngle = perAngle - this.angle;

                if (realAngle > 180){
                    realAngle = realAngle - 180;
                } else if (realAngle < 0) {
                    realAngle = realAngle + 180;
                }
            }

//            System.out.println("positionToAdd.getX(): " + positionToAdd.getX());
//            System.out.println("positionToAdd.getY(): " + positionToAdd.getY());

            double turretPosition = 77.5 + angle;

            double hypotSquared = (clawOffsetFromSlides * clawOffsetFromSlides) - (Math.abs(errors.getY()) * Math.abs(errors.getY()));

            slideOffset = Math.sqrt(hypotSquared);

            double returnTarget = ((((errors.getX()) - robotLength) - slideOffset) + 2) + positionToAdd.getX();

            if (returnTarget < 62){

                targetPositionManuel = new Vector2D(errors.getX() - robotLength, clawOffsetFromSlides - errors.getY());

                griperRotate.setPosition(realAngle);

                turret.setPosition(turretPosition);

                turretTargetPosition = turretPosition;

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

}
