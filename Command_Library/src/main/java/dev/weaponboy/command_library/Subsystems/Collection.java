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

    public SampleTargeting sampleSorter = new SampleTargeting();
    public findAngleUsingContour sampleSorterContour = new findAngleUsingContour();
    public PerspectiveTransformPipeline AntiWarp = new PerspectiveTransformPipeline();
    public VisionPortal portal;
    WebcamName closeAim;
    ElapsedTime autoCollectTimer = new ElapsedTime();
    ElapsedTime commandTimer = new ElapsedTime();
    double cameraWaitTime;

    boolean rotateWhileCollecting;

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
    boolean sensorTransfer = true;

    public boolean isSensorTransfer() {
        return sensorTransfer;
    }

    public void setSensorTransfer(boolean sensorTransfer) {
        this.sensorTransfer = sensorTransfer;
    }

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 600 / 360;
    double microRoboticTime = (double) 500 / 360;
    double gripperOpenTime = 400;

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
        transferInt,
        transfering
    }

    public enum collection{
        collecting,
        transferring,
        stowed
    }

    public enum slideState{
        manuel,
        profile
    }

    public enum clawState{
        drop,
        grab,
        slightRelease
    }

    public enum Nest{
        sample,
        specimen
    }

    enum targeting{
        camera,
        detecting,
        movingToTargeting,
        targeting,
        collecting,
        transferring
    }

    enum Obs_Collect{
        waiting,
        extendSlides,
        flipArm,
        drop
    }

    fullCommands.Obs_Collect obsCollect = fullCommands.Obs_Collect.waiting;
    //16/40

    /**
     * collect position values
     * */
    double mainPivotCollect = 78;
    double secondPivotCollect = 325;

    /**
     * preCollect position values
     * */
    double mainPivotPreCollect = 105;
    double secondPivotPreCollect = 310;
    double rotatePreCollect = 0;

    /**
     * preCollect position values
     * */
    double mainPivotMidTransfer = 210;
    double secondPivotMidTransfer = 70;
    double rotateMidTransfer = 0;

    /**
     * stowed position values
     * */
    double mainPivotStow = 180;
    double secondPivotStow = 140;

    /**
     * stowed position values
     * */
    double rotateTransInt = 0;
    double railTargetTransInt = 12.5;
    double mainPivotTransInt = 90;
    double secondPivotTransInt = 210;

    /**
     * stowed position values
     * */
    double railTargetChamberStowed = 13;
    double mainPivotChamberStowed = 210;
    double secondPivotChamberStowed = 100;

    /**
     * stowed position values
     * */
    double mainPivotTransferUp = 210;
    double secondPivotTransferUp = 100;

    /**
     * stowed position values
     * */
    double mainPivotTransfer = 200;
    double secondPivotTransfer = 125;
    double rotateTransfer = 0;

    /**
     * stow position values
     * */
    double mainPivotChamberCollect = 140;
    double secondPivotChamberCollect = 310;
    double rotateChamberCollect = 0;

    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;

    boolean autoCollecting = false;

    /**enum states*/
    private Nest nestState = Nest.sample;
    private fourBar fourBarState = fourBar.stowed;
    private fourBar fourBarTargetState = fourBar.stowed;
    private collection collectionState = collection.stowed;
    private slideState slidesState = slideState.manuel;
    targeting targetingState = targeting.camera;
    targeting targetTargetingState = targeting.camera;

    public clawState getClawsState() {
        return clawsState;
    }

    public void setClawsState(clawState clawsState) {
        this.clawsState = clawsState;
    }

    private clawState clawsState = clawState.drop;

    PIDController adjustment = new PIDController(0.015, 0, 0.00005);

    double slideTarget;
    double slideI = 0;

    public void setSlideTarget(double slideTarget) {
        if (slideTarget < 0){
            this.slideTarget = 0;
        }else {
           this.slideTarget = slideTarget;
        }

        slideI = 0;
    }

    boolean cancelTransfer = false;

    public double getSlideTarget() {
        return slideTarget;
    }

    public double slideTargetPosition;
    public double railTarget;

    boolean autoCollected = false;

    public Collection(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }

    boolean resettingSlides = false;

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
        griperRotate.setPosition(0);

        setClawsState(clawState.drop);

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

        if (fourBarState == fourBar.collect && clawSensor.isPressed() && !autoCollected && getCurrentCommand() != transfer){

            if (getChamberCollect()){
                queueCommand(chamberCollect);
            }else {
                queueCommand(transfer);
            }

            autoCollected = true;
        } else if (autoCollected && fourBarState == fourBar.preCollect) {
            autoCollected = false;
        }

        double ticksPerCM = (double) 205 / 18;
        double error;

        if ((slideTarget*ticksPerCM) < 600){
            error = Math.abs((slideTarget*ticksPerCM) - (double) horizontalMotor.getCurrentPosition());
        }else {
            error = Math.abs((600) - (double) horizontalMotor.getCurrentPosition());
        }


        if (!resettingSlides && !slidesReset.isPressed() && slideTarget == 0 && Math.abs(horizontalMotor.getVelocity()) < 10){
            slideTarget = 0;
            extendoPower = -0.6;
            resettingSlides = true;
        }else if (resettingSlides && slidesReset.isPressed() && slideTarget == 0){
            slideTarget = 0;
            extendoPower = 0;
            resettingSlides = false;

            horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (error > 5 && !resettingSlides){
            extendoPower = Range.clip(adjustment.calculate((slideTarget * ticksPerCM), horizontalMotor.getCurrentPosition()), -1, 1);
        }else if (slidesReset.isPressed()){
            extendoPower = 0;
            if(slideTarget == 0){
                resettingSlides = false;
            }
        }else {
            slideI = 0;
        }

        if (clawsState == clawState.grab){
            gripServo.setPosition(60);
        } else if (clawsState == clawState.drop) {
            gripServo.setPosition(120);
        } else if (clawsState == clawState.slightRelease) {
            gripServo.setPosition(65);
        }

        horizontalMotor.update(extendoPower);
    }

    private final Command preCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPreCollect);
                fourBarSecondPivot.setPosition(secondPivotPreCollect);
                clawsState = clawState.drop;
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
    private final Command transferUp = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotTransferUp);
                fourBarSecondPivot.setPosition(secondPivotTransferUp);
                fourBarState = fourBar.transferUp;
            }
    );
    private final Command Transfer = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotTransfer);
                fourBarSecondPivot.setPosition(secondPivotTransfer);
                clawsState = clawState.grab;
            }
    );
    private final Command transInt = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotTransInt);
                fourBarSecondPivot.setPosition(secondPivotTransInt);
                setRailTargetPosition(railTargetTransInt);
                griperRotate.setPosition(rotateTransInt);
            }
    );

    public final Command ChamberStowed = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotChamberStowed);
                fourBarSecondPivot.setPosition(secondPivotChamberStowed);
                setRailTargetPosition(railTargetChamberStowed);
            }
    );

    private final Command Stowed = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotStow);
                fourBarSecondPivot.setPosition(secondPivotStow);
            }
    );

    public final  Command slidesTargeting = new LambdaCommand(
            () -> {},
            () -> {},
            () -> Math.abs((slideTarget*((double) 205 / 18)) - (double) horizontalMotor.getCurrentPosition()) < 10

    );

    public final  Command transferDrop = new LambdaCommand(
            () -> {

            },
            () -> {
                clawsState = clawState.drop;
                TransferDrop =true;
                WaitForTranferDrop.reset();
                if (TransferDrop && WaitForTranferDrop.milliseconds()>400){
                    transferUp.execute();
                    TransferDrop = false;
                }
            },
            () -> TransferDrop && WaitForTranferDrop.milliseconds()>500

    );

    public final Command ChamberCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotChamberCollect);
                fourBarSecondPivot.setPosition(secondPivotChamberCollect);
//                griperRotate.setPosition(rotateChamberCollect);
            }
    );

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    public Command collect(double slideTarget){
        this.slideTarget = slideTarget;
        return collect;
    }

    public Command collect = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotCollect)*8);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

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
            () -> !(fourBarState == fourBar.transferringStates) && fourBarTimer.milliseconds() > transferWaitTime && fourBarState != fourBar.transferInt
    );

    public Command autoCollectGlobal(RobotPower robotPosition){
        RobotPosition = robotPosition;
        return autoCollectGlobal;
    }

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
                        System.out.println("Calculations done");

//                        if (deltaHeading == 0){
//                            targetRailPosition = 9.5;
//                            slideTarget = disToTarget-36;
//                        }else {
//                            targetRailPosition = 9.5 + (railDisToTarget * Math.sin(Math.toRadians(otherAngle)));
//
//                            slideTarget = (disToTarget - Math.abs((railDisToTarget * Math.cos((Math.toRadians(otherAngle))))))-31;
//                        }
                        railTarget = (targetRailPosition);
                        slideTargetPosition = slideTarget;

                        if (slideTarget > 50 || targetRailPosition > 26 || targetRailPosition < 0){
                            System.out.println("Out of range");
                            targetingState = targeting.collecting;

                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.stowed;
                            transferWaitTime = 10;

                        }else {

                            System.out.println("Ran Setting code");
                            setSlideTarget(slideTarget);
                            setRailTargetPosition(targetRailPosition);

                            if (sampleMap.get(0).getAngle() > 0){
                                griperRotate.setPosition(180 - (Math.abs(sampleMap.get(0).getAngle())));
                            }else {
                                griperRotate.setPosition(0 + (Math.abs(sampleMap.get(0).getAngle())));
                            }


                            double ticksPerCM = (double) 205 / 18;

                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.preCollect;
                            transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotPreCollect)*(microRoboticTime), Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*5, Math.abs((slideTarget*ticksPerCM) - horizontalMotor.getCurrentPosition())*3));

                            preCollect.execute();

                            FileWriter fWriter = null;
                            try {
                                fWriter = new FileWriter("/sdcard/VisionLogs.txt", true);

                                fWriter.write(System.lineSeparator());
                                fWriter.write(System.lineSeparator());
                                fWriter.write("current point " + new Point(RobotPosition.getVertical(), RobotPosition.getHorizontal()));
                                fWriter.write(System.lineSeparator());
                                fWriter.write("target point " + targetPointGlobal);
                                fWriter.write(System.lineSeparator());
                                fWriter.write("xError " + errors.getX());
                                fWriter.write(System.lineSeparator());
                                fWriter.write("yError " + errors.getY());
                                fWriter.write(System.lineSeparator());
                                fWriter.write("targetRailPosition " + targetRailPosition);
                                fWriter.write(System.lineSeparator());
                                fWriter.write("slideTarget " + slideTarget);
                                fWriter.write(System.lineSeparator());
                                fWriter.write("sample Angle " + sampleMap.get(0).getAngle());

                                fWriter.flush();
                            } catch (IOException e) {
                                throw new RuntimeException(e);
                            } finally {
                                if (fWriter != null) {
                                    try {
                                        fWriter.close();
                                    } catch (IOException e) {
                                        e.printStackTrace();
                                    }
                                }
                            }

                            sampleMap.remove(0);

                        }

                    }else {

                        Stowed.execute();

                        fourBarTimer.reset();
                        fourBarState = fourBar.transferringStates;
                        fourBarTargetState = fourBar.stowed;
                        transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*(microRoboticTime), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
//
//
//                        if (detectionAttemptsCounter <= 2){
//                            fourBarTimer.reset();
//                            fourBarState = fourBar.transferringStates;
//                            fourBarTargetState = fourBar.detectingSample;
//
//                            if (sampleSorterContour.isScanning() && !sampleSorterContour.detections.isEmpty()){
//                                transferWaitTime = 10;
//                                sampleSorterContour.setScanning(false);
//                            }else if (!sampleSorterContour.isScanning()) {
//                                transferWaitTime = 500;
//                                detectionAttemptsCounter++;
//                                sampleSorterContour.setScanning(true);
//                            }
//                        }else {
//
//                        }


                    }

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

                System.out.println("Still here: " + (fourBarState == fourBar.stowed && fourBarTimer.milliseconds() > transferWaitTime));
                System.out.println("Still here Collect: " + (fourBarState == fourBar.collect));

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
                        System.out.println("Calculations done");

//                        if (deltaHeading == 0){
//                            targetRailPosition = 9.5;
//                            slideTarget = disToTarget-36;
//                        }else {
//                            targetRailPosition = 9.5 + (railDisToTarget * Math.sin(Math.toRadians(otherAngle)));
//
//                            slideTarget = (disToTarget - Math.abs((railDisToTarget * Math.cos((Math.toRadians(otherAngle))))))-31;
//                        }
                        railTarget = (targetRailPosition);
                        slideTargetPosition = slideTarget;

                        if (slideTarget > 50 || targetRailPosition > 26 || targetRailPosition < 0){
                            System.out.println("Out of range");
                            targetingState = targeting.collecting;

                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.stowed;
                            transferWaitTime = 10;

                        }else {

                            System.out.println("Ran Setting code");
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

                            FileWriter fWriter = null;
                            try {
                                fWriter = new FileWriter("/sdcard/VisionLogs.txt", true);

                                fWriter.write(System.lineSeparator());
                                fWriter.write(System.lineSeparator());
                                fWriter.write("current point " + new Point(RobotPosition.getVertical(), RobotPosition.getHorizontal()));
                                fWriter.write(System.lineSeparator());
                                fWriter.write("target point " + targetPointGlobal);
                                fWriter.write(System.lineSeparator());
                                fWriter.write("xError " + errors.getX());
                                fWriter.write(System.lineSeparator());
                                fWriter.write("yError " + errors.getY());
                                fWriter.write(System.lineSeparator());
                                fWriter.write("targetRailPosition " + targetRailPosition);
                                fWriter.write(System.lineSeparator());
                                fWriter.write("slideTarget " + slideTarget);
                                fWriter.write(System.lineSeparator());
                                fWriter.write("sample Angle " + sampleMap.get(0).getAngle());

                                fWriter.flush();
                            } catch (IOException e) {
                                throw new RuntimeException(e);
                            } finally {
                                if (fWriter != null) {
                                    try {
                                        fWriter.close();
                                    } catch (IOException e) {
                                        e.printStackTrace();
                                    }
                                }
                            }

                            sampleMap.remove(0);

                        }

                    }else {

                        Stowed.execute();

                        fourBarTimer.reset();
                        fourBarState = fourBar.transferringStates;
                        fourBarTargetState = fourBar.stowed;
                        transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*(microRoboticTime), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
//
//
//                        if (detectionAttemptsCounter <= 2){
//                            fourBarTimer.reset();
//                            fourBarState = fourBar.transferringStates;
//                            fourBarTargetState = fourBar.detectingSample;
//
//                            if (sampleSorterContour.isScanning() && !sampleSorterContour.detections.isEmpty()){
//                                transferWaitTime = 10;
//                                sampleSorterContour.setScanning(false);
//                            }else if (!sampleSorterContour.isScanning()) {
//                                transferWaitTime = 500;
//                                detectionAttemptsCounter++;
//                                sampleSorterContour.setScanning(true);
//                            }
//                        }else {
//
//                        }


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

//                System.out.println("Still here: " + (fourBarState == fourBar.stowed && fourBarTimer.milliseconds() > transferWaitTime));
//                System.out.println("Still here Collect: " + (fourBarState == fourBar.collect));

            },
            () -> fourBarState == fourBar.collect || (fourBarState == fourBar.stowed && fourBarTimer.milliseconds() > transferWaitTime)
    );

    public void targetPointWithExtendo(Vector2D targetPoint){

        Vector2D errors = rotatePosition(RobotPosition.getPivot(), new Vector2D(targetPoint.getX() -RobotPosition.getVertical(), targetPoint.getY()-RobotPosition.getHorizontal()));
        double targetRailPosition;
        double slideTarget;

        double robotLength = 35 * 0.5;
        double clawOffsetFromSlides = 10.5;

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

            setSlideTarget(slideTarget);
            setRailTargetPosition(targetRailPosition);

//            double ticksPerCM = (double) 205 / 18;
//
//            fourBarTimer.reset();
//            fourBarState = fourBar.transferringStates;
//            fourBarTargetState = fourBar.preCollect;
//            transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees() - mainPivotPreCollect) * (microRoboticTime), Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees() - secondPivotPreCollect) * 5, Math.abs((slideTarget * ticksPerCM) - horizontalMotor.getCurrentPosition()) * 3));

            ChamberCollect.execute();
        }
    }

    public Command obs_Collect = new LambdaCommand(
            () -> {
                obsCollect = fullCommands.Obs_Collect.extendSlides;
            },
            () -> {

                if (obsCollect == fullCommands.Obs_Collect.extendSlides){
                    targetPointWithExtendo(new Vector2D(330.5, 63.5));
                    griperRotate.setPosition(180);
                    obsCollect = fullCommands.Obs_Collect.drop;
                    commandTimer.reset();
                }else if (obsCollect == fullCommands.Obs_Collect.drop && commandTimer.milliseconds() > 500) {
                    obsCollect = fullCommands.Obs_Collect.waiting;
                    queueCommand(collect);
                    queueCommand(collect);
//                    queueCommand(collect);
                }

//                else if (obsCollect == fullCommands.Obs_Collect.extendSlides && horizontalMotor.getVelocity() < 10){
//                    obsCollect = fullCommands.Obs_Collect.flipArm;
////                    setRailTargetPosition(18);
//                    commandTimer.reset();
//                }else if (obsCollect == fullCommands.Obs_Collect.flipArm && commandTimer.milliseconds() > 550 && commandTimer.milliseconds() < 700){
//                    setClawsState(Collection.clawState.drop);
////                    targetPointWithExtendo(new Vector2D(332, 67));
////                    setRailTargetPosition(18);
//                    commandTimer.reset();
//                    obsCollect = fullCommands.Obs_Collect.drop;
//                }
            },
            () -> obsCollect == fullCommands.Obs_Collect.waiting
    );

    public Command transfer = new LambdaCommand(
            () -> {
//                if(horizontalMotor.getCurrentPosition() > 30){
//                    profile.generateMotionProfile(0, horizontalMotor.getCurrentPosition());
//                }

                cancelTransfer = false;
            },
            () -> {

//                if (rotateWhileCollecting && fourBarState == fourBar.collect && clawsState == clawState.drop){
//
//                    fourBarTimer.reset();
//                    transferWaitTime = 1000;
//                    fourBarState = fourBar.transferringStates;
//                    fourBarTargetState = fourBar.rotateBack;
//
//                    lastRotate = griperRotate.getPositionDegrees();
//                    griperRotate.setPosition(lastRotate + 90);
//
//                }else if (rotateWhileCollecting && fourBarState == fourBar.rotateBack){
//
//                    fourBarTimer.reset();
//                    transferWaitTime = 1000;
//                    fourBarState = fourBar.transferringStates;
//                    fourBarTargetState = fourBar.collect;
//
//                    rotateWhileCollecting = false;
//
//                    griperRotate.setPosition(lastRotate + 10);
//
//                }else
//

                if ( fourBarState == fourBar.collect && clawsState == clawState.drop) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = 500;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    autoCollecting = false;
                }else if (fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotatePreCollect)*8, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransInt)*microRoboticTime);
                    fourBarTargetState = fourBar.transferUp;

                    setSlideTarget(0);
                    setClawsState(clawState.grab);

                    Transfer.execute();
                    griperRotate.setPosition(rotateTransfer);

                    setRailTargetPosition(railTargetTransInt);

                }
//                } else if (fourBarState == fourBar.collect && clawsState == clawState.grab){
//
//                    fourBarTimer.reset();
//                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*6, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransInt)*microRoboticTime);
//                    fourBarState = fourBar.transferringStates;
//                    fourBarTargetState = fourBar.transferInt;
//
//                    transInt.execute();
//                    setRailTargetPosition(railTargetTransInt);
//
//                } else if (fourBarState == fourBar.transferInt) {
//
//                    fourBarTimer.reset();
//
//                    fourBarState = fourBar.transferringStates;
//                    transferWaitTime = Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransfer)*microRoboticTime, Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotTransfer)*microRoboticTime);
//                    fourBarTargetState = fourBar.transfering;
//
//                    midTransfer.execute();
//
//                    setSlideTarget(0);
//                    setClawsState(clawState.grab);
//
//                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }
            },
            () -> (fourBarState == fourBar.transferUp || cancelTransfer)
    );

    public Command chamberCollect = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.collect && clawsState == clawState.drop){

                    fourBarTimer.reset();
                    transferWaitTime = 500;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    clawsState = clawState.grab;

                }else if (fourBarState == fourBar.collect && clawsState == clawState.grab){

                    fourBarTimer.reset();
                    transferWaitTime = 300;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collectChamber;

                    ChamberCollect.execute();

                }else if (fourBarState == fourBar.collectChamber && slideTarget != 30){

                    fourBarTimer.reset();
                    transferWaitTime = Math.abs(getSlideTarget()-30)*50;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collectChamber;

                    setSlideTarget(30);
                    setRailTargetPosition(13);

                }else if (fourBarState == fourBar.collectChamber && slideTarget == 30){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowedChamber;

                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCollect)*(microRoboticTime+10), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*10);

                    ChamberStowed.execute();

                }else if (fourBarState == fourBar.stowedChamber){

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowedChamber;

                    transferWaitTime = 0;

                    setSlideTarget(0);

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.stowedChamber && slideTarget == 0
    );

    public Command stow = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.collect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    clawsState = clawState.drop;

                    Stowed.execute();

                }else if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    clawsState = clawState.drop;

                    Stowed.execute();

                }else if (fourBarState == fourBar.collectChamber){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    clawsState = clawState.drop;

                    Stowed.execute();

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.stowed
    );

    public double getRailPosition() {
        double degreesPerCM = (double) 900 / 26;
        return (linerRailServo.getPositionDegrees() - 450)/degreesPerCM;
    }

    public void setTargetPoint(Point targetPoint){
        this.targetPointGlobal = targetPoint;
    }

    public void setRailTargetPosition(double targetPosition) {
        this.railTargetPosition = targetPosition;
        double degreesPerCM = (double) 900 / 26;

        double servoTarget = 450+(degreesPerCM*targetPosition);

        if (servoTarget < 450){
            linerRailServo.setPosition(450);
        } else if (servoTarget > 1350) {
            linerRailServo.setPosition(1350);
        }else {
            linerRailServo.setPosition(servoTarget);
        }

    }

    public fourBar getFourBarState() {
        return fourBarState;
    }

    public collection getCollectionState() {
        return collectionState;
    }

    public Collection.slideState getSlideState() {
        return slidesState;
    }

    public Nest getNestState() {
        return nestState;
    }

    public void setNestState(Nest nestState) {
        this.nestState = nestState;
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

    public double getSlidePositionCM(){
        double ticksPerCM = (double) 18 / 205;
        return horizontalMotor.getCurrentPosition() * ticksPerCM;
    }

}
