package dev.weaponboy.command_library.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.Execute;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
import dev.weaponboy.vision.SamplePipelines.PerspectiveTransformPipeline;
import dev.weaponboy.vision.detectionData;
import dev.weaponboy.command_library.Hardware.motionProfile;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.vision.SamplePipelines.SampleTargeting;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;

public class Collection extends SubSystem {

    public ArrayList<detectionData> sampleMap = new ArrayList<>();

    public findAngleUsingContour sampleSorterContour = new findAngleUsingContour();
    public VisionPortal portal;

    WebcamName closeAim;
    ElapsedTime commandTimer = new ElapsedTime();
    boolean transferToFar = false;

    // slides
    public MotorEx horizontalMotor = new MotorEx();
    double extendoPower = 0;
    motionProfile profile = new motionProfile(508, 350, 50, 492, 0.15);

    //servos
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoDegrees gripServo = new ServoDegrees();
    public ServoDegrees linerRailServo=new ServoDegrees();

    public TouchSensor clawSensor;
    public TouchSensor slidesReset;

    /**
     * linear rail constants
     * */
    double railTargetPosition;
    double railWaitTime;
    ElapsedTime railWait = new ElapsedTime();

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 600 / 360;
    double microRoboticTime = (double) 500 / 360;
    double gripperOpenTime = 300;

    boolean secondPivot = false;

    ElapsedTime secondPivotTime = new ElapsedTime();

    double robotLength = 35 * 0.5;
    double clawOffsetFromSlides = 10.5;

    public boolean getChamberCollect() {
        return chamberCollectBool;
    }

    public void setChamberCollect(boolean chamberCollectBool) {
        this.chamberCollectBool = chamberCollectBool;
    }

    boolean chamberCollectBool = false;

    public AxonEncoder linearPosition = new AxonEncoder();

    boolean TransferDrop =false;
    ElapsedTime WaitForTranferDrop = new ElapsedTime();

    /**states*/
    public enum fourBar{
        preCollect,
        preCollectFlipOut,
        collect,
        detectingSample,
        transferringStates,
        collectChamber,
        stowedChamber,
        transferUp,
        stowed,
        wallCollect,
        wallRotate,
        transferInt
    }

    public enum slideState{
        manuel,
        profile
    }

    public enum clawState{
        drop,
        grab,
        slightRelease,
        openFull
    }

    fullCommands.Obs_Collect obsCollect = fullCommands.Obs_Collect.waiting;

    /**
     * collect position values
     * */
    double mainPivotCollect = 72;
    double secondPivotCollect = 320;

    /**
     * preCollect position values
     * */
    double mainPivotPreCollect = 105;
    double secondPivotPreCollect = 305;

    /**
     * collect position values
     * */
    double mainPivotSpikePush = 65;
    double secondPivotSpikePush = 325;

    /**
     * preCollect position values
     * */
    double mainPivotMidTransfer = 170;
    double secondPivotMidTransfer = 165;

    /**
     * preCollect position values
     * */
    double mainPivotWallCollect = 125;
    double secondPivotWallCollect = 245;

    /**
     * stowed position values
     * */
    double mainPivotStow = 180;
    double secondPivotStow = 130;

    /**
     * stowed position values
     * */
    double rotateTransInt = 90;
    double railTargetTransInt = 12.5;
    double mainPivotTransInt = 100;
    double secondPivotTransInt = 210;

    /**
     * stowed position values
     * */
    double railTargetChamberStowed = 10;
    double mainPivotChamberStowed = 160;
    double secondPivotChamberStowed = 240;

    /**
     * stowed position values
     * */
    double mainPivotTransferUp = 210;
    double secondPivotTransferUp = 90;

    /**
     * stowed position values
     * */
    double mainPivotTransfer = 198;
    double secondPivotTransfer = 139.7;
    double rotateTransfer = 180;

    /**
     * stow position values
     * */
    double mainPivotChamberCollect = 140;
    double secondPivotChamberCollect = 310;

    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;

    boolean autoCollecting = false;

    /**enum states*/
    private fourBar fourBarState = fourBar.stowed;
    private fourBar fourBarTargetState = fourBar.stowed;
    private slideState slidesState = slideState.manuel;

    public clawState getClawsState() {
        return clawsState;
    }

    public void setClawsState(clawState clawsState) {
        this.clawsState = clawsState;
    }

    private clawState clawsState = clawState.drop;

    PIDController adjustment = new PIDController(0.015, 0, 0.05);

    PIDController adjustmentClose = new PIDController(0.009, 0, 0.009);

    double slideTarget;
    double slideI = 0;
    boolean longTarget = false;

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

        slideI = 0;
    }

    boolean cancelTransfer = false;

    boolean braking = false;
    ElapsedTime brakingTimer = new ElapsedTime();

    public void setBraking(boolean braking) {
        this.braking = braking;
        brakingTimer.reset();
    }

    public double getSlideTarget() {
        return slideTarget;
    }

    public double slideTargetPosition;
    public double railTarget;

    boolean autoCollected = false;

    public Collection(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }

    public boolean resettingSlides = false;
    public boolean resetUsingCurrent = false;

    RobotPower RobotPosition = new RobotPower();
    public Point targetPointGlobal;
    public double angle;
    int detectionAttemptsCounter = 0;

    @Override
    public void init() {
        horizontalMotor.initMotor("horizontalMotor", getOpModeEX().hardwareMap);

        fourBarMainPivot.initServo("fourBarMainPivot", getOpModeEX().hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot", getOpModeEX().hardwareMap);
        griperRotate.initServo("gripperRotate", getOpModeEX().hardwareMap);
        gripServo.initServo("gripServo", getOpModeEX().hardwareMap);
        linerRailServo.initServo("linearRailServo", getOpModeEX().hardwareMap);
        linearPosition.init(getOpModeEX().hardwareMap, "axon2");

        clawSensor = getOpModeEX().hardwareMap.get(TouchSensor.class, "clawsensor");

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(335);
        linerRailServo.setRange(1800);
        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);
        gripServo.setRange(180);

        slidesReset = getOpModeEX().hardwareMap.get(TouchSensor.class, "CollectionReset");

        closeAim = getOpModeEX().hardwareMap.get(WebcamName.class, "webcam");
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(closeAim);
        builder.addProcessor(sampleSorterContour);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(1280, 960));
        portal = builder.build();

        profile.isVertical(false);

        fourBarMainPivot.setOffset(10);
        fourBarSecondPivot.setOffset(-20);

        griperRotate.setDirection(Servo.Direction.REVERSE);
        griperRotate.setOffset(10);
        griperRotate.setPosition(180);

        setClawsState(clawState.drop);
        gripServo.setPosition(90);

        Stowed.execute();

        //max left = 450
        //max right = 1350
//        linerRailServo.setDirection(Servo.Direction.REVERSE);
//        linerRailServo.setPosition(450 + (double) (1350 - 450) /2);
        setRailTargetPosition(13);
    }

    @Override
    public void execute() {

        executeEX();
//
//        if (fourBarState == fourBar.collect && clawSensor.isPressed() && !autoCollected && getCurrentCommand() != transfer){
//
//            autoCollected = true;
//        } else if (autoCollected && fourBarState == fourBar.preCollect) {
//            autoCollected = false;
//        }

        double ticksPerCM = (double) 205 / 18;
        double error;

        if ((slideTarget*ticksPerCM) < 600){
            error = Math.abs((slideTarget*ticksPerCM) - (double) horizontalMotor.getCurrentPosition());
        }else {
            error = Math.abs((600) - (double) horizontalMotor.getCurrentPosition());
        }

        if (!resettingSlides && !slidesReset.isPressed() && slideTarget == 0 && Math.abs(horizontalMotor.getVelocity()) < 10 && horizontalMotor.getCurrentPosition() < 100){
            slideTarget = 0;
            extendoPower = -0.55;
            resettingSlides = true;
        }else if (resettingSlides && slidesReset.isPressed()){
            slideTarget = 0;
            extendoPower = 0;
            resettingSlides = false;

            horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

//        else if (resettingSlides && !slidesReset.isPressed() && horizontalMotor.getCurrentDraw() > 2500){
//            slideTarget = 0;
//            extendoPower = 0;
//            resettingSlides = false;
//
//            resetUsingCurrent = true;
//
//            horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }

        if (slidesReset.isPressed() && slideTarget == 0){
            extendoPower = 0;
        }else if (error > 2 && !resettingSlides){

            if (longTarget){
                extendoPower = Range.clip(adjustment.calculate((slideTarget * ticksPerCM), horizontalMotor.getCurrentPosition()), -1, 1);
            }else{
                extendoPower = Range.clip(adjustmentClose.calculate((slideTarget * ticksPerCM), horizontalMotor.getCurrentPosition()), -1, 1);
            }

        }else if (slidesReset.isPressed()){
            extendoPower = 0;
            if(slideTarget == 0){
                resettingSlides = false;
            }
        }else {
            slideI = 0;
        }

        if (error < 10 && longTarget && slideTarget != 0){
            longTarget = false;
        }else if (slideTarget == 0 && horizontalMotor.getVelocity() < 10 && longTarget){
            longTarget = false;
        }

        if (clawsState == clawState.grab){
            gripServo.setPosition(47);
        } else if (clawsState == clawState.drop){
            gripServo.setPosition(108);
        } else if (clawsState == clawState.slightRelease){
            gripServo.setPosition(58);
        }else if (clawsState == clawState.openFull){
            gripServo.setPosition(108);
        }

        horizontalMotor.update(Range.clip(extendoPower, -1, 0.8));

//        if (!braking){
//
//        }else{
//            if (brakingTimer.milliseconds() > 100){
//                braking = false;
//            }
//            horizontalMotor.update(-(horizontalMotor.getVelocity()/ticksPerCM));
//        }

    }

    private final Command WallCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotWallCollect);
                fourBarSecondPivot.setPosition(secondPivotWallCollect);
                griperRotate.setPosition(0);
            }
    );

    private final Command preCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPreCollect);
                fourBarSecondPivot.setPosition(secondPivotPreCollect);

            }
    );

    private final Command midTransfer = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotMidTransfer);
                fourBarSecondPivot.setPosition(secondPivotMidTransfer);
            }
    );

    private final Command Collect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotCollect);
                fourBarSecondPivot.setPosition(secondPivotCollect);
            }
    );

    private final Command SpikePushing = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotSpikePush);
                fourBarSecondPivot.setPosition(secondPivotSpikePush);
                clawsState = clawState.openFull;
            }
    );

    private final Command Transfer = new Execute(
            () -> {
                fourBarSecondPivot.setPosition(secondPivotTransfer);
                fourBarMainPivot.setPosition(mainPivotTransfer);

                clawsState = clawState.grab;
            }
        );

    public final Command ChamberStowed = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotChamberStowed);
                fourBarSecondPivot.setPosition(secondPivotChamberStowed);
                setRailTargetPosition(railTargetChamberStowed);
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

    public final Command slidesTargeting = new LambdaCommand(
            () -> {},
            () -> {},
            () -> Math.abs((slideTarget*((double) 205 / 18)) - (double) horizontalMotor.getCurrentPosition()) < 10
    );

    public final Command transferDrop = new LambdaCommand(
            () -> {
                Transfer.execute();
                setClawsState(clawState.slightRelease);
                WaitForTranferDrop.reset();
                TransferDrop = false;
                fourBarState = fourBar.stowed;
            },
            () -> {
                if (WaitForTranferDrop.milliseconds() > 100){
                    TransferDrop = true;
                }
            },
            () -> TransferDrop

    );

    public final Command openGripper = new LambdaCommand(
            () -> {
                setClawsState(clawState.drop);
                Stowed.execute();
            },
            () -> {
            },
            () -> true

    );

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
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
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCollect)*4, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*6);

                    Collect.execute();

                } else if (fourBarState == fourBar.collect) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*6);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;

                    preCollect.execute();

                }else if (!(fourBarState == fourBar.transferringStates)){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*microRoboticTime);
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

    public Command preCollectNoRotate(double rotateTarget){
        griperRotate.setPosition(rotateTarget);
        return preCollectNoRotate;
    }

    public Command preCollectNoRotate = new LambdaCommand(
            () -> {},
            () -> {

                  if (!(fourBarState == fourBar.transferringStates)){

                        fourBarTimer.reset();
                        transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotPreCollect)*microRoboticTime);
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

    public Command spikePushing(double RotateTarget){
        griperRotate.setPosition(RotateTarget);
        return spikePushing;
    }

    private final Command spikePushing = new LambdaCommand(
            () -> {},
            () -> {

                if (!(fourBarState == fourBar.transferringStates)){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotSpikePush)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotSpikePush)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    SpikePushing.execute();

                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> !(fourBarState == fourBar.transferringStates) && fourBarTimer.milliseconds() > transferWaitTime && fourBarState != fourBar.transferInt
    );

    public Command extendoTargetPoint(Point targetPoint){
        this.targetPointGlobal = targetPoint;
        return extendoTargetPoint;
    }

    private Command extendoTargetPoint = new LambdaCommand(
            () -> {
                fourBarTimer.reset();
            },
            () -> {

                Vector2D errors = rotatePosition(RobotPosition.getPivot(), new Vector2D(targetPointGlobal.x -RobotPosition.getVertical(), targetPointGlobal.y-RobotPosition.getHorizontal()));
                double targetRailPosition;
                double slideTarget;

                targetRailPosition = 11 + errors.getY();
                slideTarget = (errors.getX() - robotLength)-clawOffsetFromSlides;

                railTarget = (targetRailPosition);
                slideTargetPosition = slideTarget;

                if (slideTarget > 60 || targetRailPosition > 26 || targetRailPosition < 0){

                }else {
                    setSlideTarget(slideTarget);
                    setRailTargetPosition(targetRailPosition);
                }

            },
            () -> Math.abs(getSlideTarget() - getSlidePositionCM()) < 1.1 && !isRailMoving() && Math.abs(horizontalMotor.getVelocity()) < 5 && Math.abs(extendoPower) < 0.2
    );

    public void updateRobotPosition(RobotPower robotPosition){
        RobotPosition = robotPosition;
    }

    public final Command autoCollectGlobal = new LambdaCommand(
            () -> {

                detectionAttemptsCounter = 0;
                fourBarTimer.reset();
                fourBarState = fourBar.transferringStates;
                fourBarTargetState = fourBar.detectingSample;

                transferWaitTime = 10;
            },
            () -> {

                if (fourBarState == fourBar.detectingSample){

                    sampleSorterContour.setScanning(false);

                    boolean runCollection = false;
                    double rotateAngle = 0;

                    while (!sampleMap.isEmpty() && !runCollection){

                        angle = sampleMap.get(0).getAngle();

                        targetPointGlobal = sampleMap.get(0).getTargetPoint();

                        rotateAngle = sampleMap.get(0).getAngle();

                        sampleMap.remove(0);

                        Vector2D errors = rotatePosition(RobotPosition.getPivot(), new Vector2D(targetPointGlobal.x -RobotPosition.getVertical(), targetPointGlobal.y-RobotPosition.getHorizontal()));
                        double targetRailPosition;
                        double slideTarget;

                        targetRailPosition = 11 + errors.getY();
                        slideTarget = (errors.getX() - robotLength)-clawOffsetFromSlides;

                        railTarget = (targetRailPosition);
                        slideTargetPosition = slideTarget;

                        if (slideTarget > 60 || targetRailPosition > 26 || targetRailPosition < 0){

                        }else {
                            runCollection = true;
                        }

                    }

                    if (!runCollection){

                        Stowed.execute();

                        fourBarTimer.reset();
                        fourBarState = fourBar.transferringStates;
                        fourBarTargetState = fourBar.stowed;
                        transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*(microRoboticTime), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);

                    }else {
                        preCollect.execute();
                        fourBarState = fourBar.preCollect;

                        if (rotateAngle > 80 || rotateAngle < -80){
                            griperRotate.setPosition(rotateTransfer);
                        }else{
                            griperRotate.setPosition(90 - ((rotateAngle)));
                        }

                        queueCommand(extendoTargetPoint(targetPointGlobal));
                        queueCommand(collect);
                    }

                } else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.preCollect || (fourBarState == fourBar.stowed && fourBarTimer.milliseconds() > transferWaitTime)
    );

    public final Command autoCollectChamber = new LambdaCommand(
            () -> {

                detectionAttemptsCounter = 0;
                fourBarTimer.reset();
                fourBarState = fourBar.transferringStates;
                fourBarTargetState = fourBar.detectingSample;

                transferWaitTime = 10;

            },
            () -> {

                if (fourBarState == fourBar.detectingSample){

                    sampleSorterContour.setScanning(false);

                    if (!sampleMap.isEmpty()){

                        angle = sampleMap.get(0).getAngle();

                        targetPointGlobal = sampleMap.get(0).getTargetPoint();

                        Vector2D errors = rotatePosition(RobotPosition.getPivot(), new Vector2D(targetPointGlobal.x-RobotPosition.getVertical(), targetPointGlobal.y-RobotPosition.getHorizontal()));
                        double targetRailPosition;
                        double slideTarget;

                        double robotLength = 35 * 0.5;
                        double clawOffsetFromSlides = 8;

                        targetRailPosition = 11 + errors.getY();
                        slideTarget = (errors.getX() - robotLength)-clawOffsetFromSlides;

                        railTarget = (targetRailPosition);
                        slideTargetPosition = slideTarget;

                        if (slideTarget > 50 || targetRailPosition > 26 || targetRailPosition < 0){

                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.stowed;
                            transferWaitTime = 10;

                        }else {

                            setSlideTarget(slideTarget);
                            setRailTargetPosition(targetRailPosition);

                            griperRotate.setPosition(90 - ((sampleMap.get(0).getAngle())));

                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.preCollectFlipOut;
                            transferWaitTime = 150;

                            fourBarMainPivot.setPosition(mainPivotTransInt);

                            sampleMap.remove(0);

                        }

                    }else {

                        Stowed.execute();

                        fourBarTimer.reset();
                        fourBarState = fourBar.transferringStates;
                        fourBarTargetState = fourBar.stowed;
                        transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*(microRoboticTime), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);

                    }

                }else if (fourBarState == fourBar.preCollectFlipOut){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotTransInt)*(microRoboticTime+2), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransInt)*2);

                    fourBarSecondPivot.setPosition(190);

                }else if (fourBarState == fourBar.transferInt){

                    double ticksPerCM = (double) 205 / 18;

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotPreCollect)*(microRoboticTime+2), Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*5, Math.abs((slideTarget*ticksPerCM) - horizontalMotor.getCurrentPosition())*3));

                    preCollect.execute();

                }else if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCollect)*(microRoboticTime+10), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*10);

                    Collect.execute();

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.collect || (fourBarState == fourBar.stowed && fourBarTimer.milliseconds() > transferWaitTime)
    );

    public final Command obsCollectGlobal = new LambdaCommand(
            () -> {

                detectionAttemptsCounter = 0;
                fourBarTimer.reset();
                fourBarState = fourBar.transferringStates;
                fourBarTargetState = fourBar.detectingSample;

                transferWaitTime = 10;

//                if (sampleSorterContour.isScanning() && !sampleSorterContour.detections.isEmpty()){
//                    transferWaitTime = 10;
//                    sampleSorterContour.setScanning(false);
//                }else if (!sampleSorterContour.isScanning()) {
//                    transferWaitTime = 500;
//                    sampleSorterContour.setScanning(true);
//                    detectionAttemptsCounter++;
//                }

            },
            () -> {

                if (fourBarState == fourBar.detectingSample){

                    sampleSorterContour.setScanning(false);

                    if (!sampleMap.isEmpty()){

//                        sampleMap = sampleSorterContour.convertPositionsToFieldPositions(RobotPosition);
                        angle = sampleMap.get(0).getAngle();

                        targetPointGlobal = sampleMap.get(0).getTargetPoint();

                        double angle = findAngle(targetPointGlobal, new Point(RobotPosition.getVertical(), RobotPosition.getHorizontal()));

                        Vector2D errors = rotatePosition(RobotPosition.getPivot(), new Vector2D(targetPointGlobal.x-RobotPosition.getVertical(), targetPointGlobal.y-RobotPosition.getHorizontal()));
                        double targetRailPosition;
                        double slideTarget;

                        double robotLength = 35 * 0.5;
                        double clawOffsetFromSlides = 10.5;

                        targetRailPosition = 13 + errors.getY();
                        slideTarget = (errors.getX() - robotLength)-clawOffsetFromSlides;

                        railTarget = (targetRailPosition);
                        slideTargetPosition = slideTarget;

                        if (slideTarget > 50 || targetRailPosition > 26 || targetRailPosition < 0){

                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.stowed;
                            transferWaitTime = 10;

                        }else {

                            setSlideTarget(slideTarget);

                            if (sampleMap.get(0).getAngle() > 0){
                                griperRotate.setPosition(180 - (Math.abs(sampleMap.get(0).getAngle())));
                            }else {
                                griperRotate.setPosition(0 + (Math.abs(sampleMap.get(0).getAngle())));
                            }


                            double ticksPerCM = (double) 205 / 18;

                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.stowedChamber;
                            transferWaitTime = Math.max(Math.abs(getRailPosition()-3)*(microRoboticTime), Math.abs((slideTarget*ticksPerCM) - horizontalMotor.getCurrentPosition())*3);

                            setRailTargetPosition(3);
                            ChamberStowed.execute();

                            sampleMap.remove(0);

                        }

                    }else {

                        Stowed.execute();

                        fourBarTimer.reset();
                        fourBarState = fourBar.transferringStates;
                        fourBarTargetState = fourBar.stowed;
                        transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*(microRoboticTime), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);

                    }

                } else if (fourBarState == fourBar.stowedChamber && clawsState == clawState.grab){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;
                    transferWaitTime = 500;

                    clawsState = clawState.drop;

                } else if (fourBarState == fourBar.stowedChamber && clawsState == clawState.drop){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotPreCollect)*(10), Math.abs(getRailPosition()-railTarget)*30);

                    setRailTargetPosition(railTarget);
                    preCollect.execute();

                }else if (fourBarState == fourBar.preCollect){

                    Collect.execute();

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCollect)*(microRoboticTime+10), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*10);

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.collect || (fourBarState == fourBar.stowed && fourBarTimer.milliseconds() > transferWaitTime)
    );

    public void targetPointWithExtendo(Vector2D targetPoint){

        Vector2D errors = rotatePosition(RobotPosition.getPivot(), new Vector2D(targetPoint.getX() -RobotPosition.getVertical(), targetPoint.getY()-RobotPosition.getHorizontal()));
        double targetRailPosition;
        double slideTarget;

        targetRailPosition = 13 + errors.getY();
        slideTarget = (errors.getX() - robotLength)-clawOffsetFromSlides;

        railTarget = (targetRailPosition);
        slideTargetPosition = slideTarget;

        if (slideTarget > 50 || targetRailPosition > 26 || targetRailPosition < 0){

//            targetingState = targeting.collecting;
//
//            fourBarTimer.reset();
//            fourBarState = fourBar.transferringStates;
//            fourBarTargetState = fourBar.stowed;
//            transferWaitTime = 10;

        }else {

            fourBarMainPivot.setPosition(mainPivotChamberCollect);
            fourBarSecondPivot.setPosition(secondPivotChamberCollect);

            setSlideTarget(slideTarget);
            setRailTargetPosition(targetRailPosition);

//            double ticksPerCM = (double) 205 / 18;
//
//            fourBarTimer.reset();
//            fourBarState = fourBar.transferringStates;
//            fourBarTargetState = fourBar.preCollect;
//            transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees() - mainPivotPreCollect) * (microRoboticTime), Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees() - secondPivotPreCollect) * 5, Math.abs((slideTarget * ticksPerCM) - horizontalMotor.getCurrentPosition()) * 3));

        }
    }

    public void targetPointWithExtendoNoArm(Vector2D targetPoint){

        Vector2D errors = rotatePosition(RobotPosition.getPivot(), new Vector2D(targetPoint.getX() -RobotPosition.getVertical(), targetPoint.getY()-RobotPosition.getHorizontal()));
        double targetRailPosition;
        double slideTarget;

        targetRailPosition = 13 + errors.getY();
        slideTarget = (errors.getX() - robotLength)-clawOffsetFromSlides;

        railTarget = (targetRailPosition);
        slideTargetPosition = slideTarget;

        if (slideTarget > 50 || targetRailPosition > 26 || targetRailPosition < 0){

        }else {
            setSlideTarget(slideTarget);
            setRailTargetPosition(targetRailPosition);
        }
    }

    public Vector2D extendoPoint(){
        Vector2D currentPoint;

        double yPos = getRailPosition() - 13;
        double xPos = getSlidePositionCM()+clawOffsetFromSlides+robotLength;

        Vector2D errors = rotatePositionToGlobal(RobotPosition.getPivot(), new Vector2D(xPos, yPos));

        currentPoint = new Vector2D(RobotPosition.getVertical()+errors.getX(), RobotPosition.getHorizontal()+errors.getY());

        return currentPoint;
    }

    public Command obs_Collect = new LambdaCommand(
            () -> {
                obsCollect = fullCommands.Obs_Collect.drop;
            },
            () -> {

                if (obsCollect == fullCommands.Obs_Collect.drop){
                    obsCollect = fullCommands.Obs_Collect.positionToGrab;
                    clawsState = clawState.drop;
                    commandTimer.reset();
                } else if (obsCollect == fullCommands.Obs_Collect.positionToGrab && commandTimer.milliseconds() > 400){
                    targetPointWithExtendo(new Vector2D(332.5, 61));
                    griperRotate.setPosition(180);
                    obsCollect = fullCommands.Obs_Collect.collect;
                    commandTimer.reset();
                }else if (obsCollect == fullCommands.Obs_Collect.collect && commandTimer.milliseconds() > 800) {
                    obsCollect = fullCommands.Obs_Collect.waiting;
                    setChamberCollect(false);
                    queueCommand(collect);
                    queueCommand(collect);
                }

            },
            () -> obsCollect == fullCommands.Obs_Collect.waiting
    );

    public Command obs_Collect_No_Drop = new LambdaCommand(
            () -> {
                obsCollect = fullCommands.Obs_Collect.positionToGrab;
                clawsState = clawState.drop;
                commandTimer.reset();
            },
            () -> {

                if (obsCollect == fullCommands.Obs_Collect.positionToGrab && commandTimer.milliseconds() > 200){
                    targetPointWithExtendo(new Vector2D(332.5, 61));
                    griperRotate.setPosition(180);
                    obsCollect = fullCommands.Obs_Collect.collect;
                    commandTimer.reset();
                }else if (obsCollect == fullCommands.Obs_Collect.collect && commandTimer.milliseconds() > 800) {
                    obsCollect = fullCommands.Obs_Collect.waiting;
                    setChamberCollect(false);
                    queueCommand(collect);
                    queueCommand(collect);
                }
            },
            () -> obsCollect == fullCommands.Obs_Collect.waiting
    );


    public Command transfer = new LambdaCommand(
            () -> {
                cancelTransfer = false;
            },
            () -> {

                if (fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 5) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    autoCollecting = false;

                }else if (fourBarState == fourBar.collect && clawsState == clawState.grab && (getRailPosition() > 16 || getRailPosition() < 10)){

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*microRoboticTime, Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*microRoboticTime, Math.abs(getRailPosition() - railTargetTransInt)*8));
                    fourBarTargetState = fourBar.collect;

                    fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                    fourBarSecondPivot.setPosition(secondPivotPreCollect);
//                    setClawsState(clawState.grab);

                    griperRotate.setPosition(rotateTransfer);
                    setRailTargetPosition(railTargetTransInt);

                } else if (fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*4, Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*2, Math.abs(getRailPosition() - railTargetTransInt)*8));
                    fourBarTargetState = fourBar.transferUp;

                    setSlideTarget(0);
                    setClawsState(clawState.grab);

                    if (horizontalMotor.getCurrentPosition() < 320){
                        Transfer.execute();
                        commandTimer.reset();
                    }else{
//                        preCollect.execute();
                        fourBarMainPivot.setPosition(mainPivotCollect+20);
                        transferToFar = true;
                    }

                    griperRotate.setPosition(rotateTransfer);

                    setRailTargetPosition(railTargetTransInt);

                }

                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
                    Transfer.execute();
                    transferToFar = false;
                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }
            },
            () -> (fourBarState == fourBar.transferUp || cancelTransfer) && slidesReset.isPressed()
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
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*5, Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*2, Math.abs(getRailPosition() - railTargetTransInt)*2));
                    fourBarTargetState = fourBar.wallRotate;

                    griperRotate.setPosition(rotateTransfer);
                    setRailTargetPosition(railTargetTransInt);

                } else if (fourBarState == fourBar.wallRotate) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotTransfer)*(microRoboticTime+10), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransfer)*10);
                    fourBarTargetState = fourBar.transferUp;

                    setSlideTarget(0);

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
            () -> {},
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
                    griperRotate.setPosition(90);
                    setSlideTarget(4);

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.stowedChamber && slideTarget == 4
    );

    public Command stow = new LambdaCommand(
            () -> {
                clawsState = clawState.drop;
            },
            () -> {

                if (fourBarState != fourBar.transferringStates){
                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    clawsState = clawState.drop;
                    Stowed.execute();
                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> fourBarState == fourBar.stowed
    );

    public double getRailPosition() {
        double degreesPerCM = (double) 900 / 26;
        return (linerRailServo.getPositionDegrees() - 425)/degreesPerCM;
    }

    public void setRailTargetPosition(double targetPosition) {
        if(Math.abs(getRailPosition() - railTargetPosition) > 1){
            railWaitTime = Math.abs(getRailPosition() - railTargetPosition)*70;
            railWait.reset();
        }

        this.railTargetPosition = targetPosition;
        double degreesPerCM = (double) 900 / 26;

        double servoTarget = 425+(degreesPerCM*targetPosition);

        if (servoTarget < 425){
            linerRailServo.setPosition(425);
        } else if (servoTarget > 1375) {
            linerRailServo.setPosition(1375);
        }else {
            linerRailServo.setPosition(servoTarget);
        }

    }

    public boolean isRailMoving(){
        return railWaitTime > railWait.milliseconds();
    }

    public fourBar getFourBarState() {
        return fourBarState;
    }

    public void disableServos(){
        fourBarMainPivot.disableServo();
        fourBarSecondPivot.disableServo();
        griperRotate.disableServo();
        gripServo.disableServo();
        linerRailServo.disableServo();
    }

    public double findAngle(Point targetPoint, Point currentPoint){
        double xError = targetPoint.x-currentPoint.x;
        double yError = targetPoint.y-currentPoint.y;

        double slope = Math.atan(yError/xError);

        double degrees = 0;

        if (xError >= 0 && yError <= 0){
            degrees = (90-(-Math.toDegrees(slope)))+270;
        }else if (xError <= 0 && yError >= 0){
            degrees = (90-(-Math.toDegrees(slope)))+90;
        }else if (xError <= 0 && yError <= 0){
            degrees = Math.toDegrees(slope)+180;
        }else {
            degrees = Math.toDegrees(slope);
        }

        return degrees;
    }

    public static Vector2D rotatePosition(double heading, Vector2D point){
        double X = (point.getY()) * Math.sin(Math.toRadians(heading)) + (point.getX()) * Math.cos(Math.toRadians(heading));
        double Y = (point.getY()) * Math.cos(Math.toRadians(heading)) - (point.getX()) * Math.sin(Math.toRadians(heading));

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

}
