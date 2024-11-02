package dev.weaponboy.command_library.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.MandatoryWarningHandler;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.io.FileWriter;
import java.io.IOException;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.Execute;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.command_library.Hardware.motionProfile;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.vision.SamplePipelines.SampleTargeting;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;

public class Collection extends SubSystem {

    public SampleTargeting sampleSorter = new SampleTargeting();
    public findAngleUsingContour sampleSorterContour = new findAngleUsingContour();
    public VisionPortal portal;
    WebcamName closeAim;
    ElapsedTime autoCollectTimer = new ElapsedTime();
    double cameraWaitTime;

    boolean rotateWhileCollecting;

    enum targeting{
        camera,
        detecting,
        movingToTargeting,
        targeting,
        collecting,
        transferring
    }

    targeting targetingState = targeting.camera;
    targeting targetTargetingState = targeting.camera;

    // slides
    public MotorEx horizontalMotor = new MotorEx();
    double extendoPower = 0;
    motionProfile profile = new motionProfile(508, 100, 35, 440, 0.15);

    //servos
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoImplEx gripServo;
    public ServoDegrees linerRailServo=new ServoDegrees();
    public ServoDegrees nest = new ServoDegrees();

    public TouchSensor nestSensor;
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
    double axonMaxTime = (double) 750 / 360;
    double microRoboticTime = (double) 900 / 360;
    double gripperOpenTime = 400;

    public boolean getChamberCollect() {
        return chamberCollectBool;
    }

    public void setChamberCollect(boolean chamberCollectBool) {
        this.chamberCollectBool = chamberCollectBool;
    }

    boolean chamberCollectBool = false;

    public AxonEncoder linearPosition = new AxonEncoder();

    /**states*/
    public enum fourBar{
        preCollect,
        collect,
        rotateBack,
        dropNest,
        transferringStates,
        transferUp,
        aboveNest,
        collectChamber,
        stowed,
        transferInt
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

    /**
     * collect position values
     * */
    double mainPivotCollect = 93;
    double secondPivotCollect = 10;

    /**
     * preCollect position values
     * */
    double mainPivotPreCollect = 116;
    double secondPivotPreCollect = 65;
    double rotatePreCollect = 90;

    /**
     * transfer up position values
     * */
    double mainPivotTransferUp = 154;
    double secondPivotTransferUp = 175;

    /**
     * stowed position values
     * */
    double mainPivotStow = 165;
    double secondPivotStow = 195;

    /**
     * stow position values
     * */
    double mainPivotTransInt = 90;
    double secondPivotTransInt = 140;
    double railTargetTransInt = 9.2;
    double rotateTransInt = 180;

    /**
     * stow position values
     * */
    double mainPivotCamera = 170;
    double secondPivotCamera = 118;

    /**
     * stow position values
     * */
    double mainPivotAboveNest = 185;
    double secondPivotAboveNest = 200;

    /**
     * drop nest position values
     * */
    double mainPivotPlaceNest = 200;
    double secondPlaceNest = 215;
    double rotatePlaceNest = 90;

    /**
     * stow position values
     * */
    double mainPivotChamberCollect = 145;
    double secondPivotChamberCollect = 70;
    double rotateChamberCollect = 90;

    /**
     * drop nest above position values
     * */
    double mainPivotDropNestPull = 170;
    double secondDropNestPull = 210;
    double rotateDropNestPull = 90;

    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;

    boolean autoCollecting = false;

    /**enum states*/
    private Nest nestState = Nest.sample;
    private fourBar fourBarState = fourBar.transferUp;
    private fourBar fourBarTargetState = fourBar.transferUp;
    private collection collectionState = collection.stowed;
    private slideState slidesState = slideState.manuel;

    public clawState getClawsState() {
        return clawsState;
    }

    public void setClawsState(clawState clawsState) {
        this.clawsState = clawsState;
    }

    private clawState clawsState = clawState.drop;

    PIDController adjustment = new PIDController(0.002, 0, 0.00001);

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

    static FileWriter fWriter;

    ElapsedTime slidesResetTimer = new ElapsedTime();
    boolean resettingSlides = false;

    static {
        try {
            fWriter = new FileWriter("/sdcard/railLogs.txt");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    RobotPower RobotPosition = new RobotPower();
    public Point targetPointGlobal;

    @Override
    public void init() {
        horizontalMotor.initMotor("horizontalMotor", getOpModeEX().hardwareMap);

        fourBarMainPivot.initServo("fourBarMainPivot", getOpModeEX().hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot", getOpModeEX().hardwareMap);
        griperRotate.initServo("gripperRotate", getOpModeEX().hardwareMap);
        gripServo = getOpModeEX().hardwareMap.get(ServoImplEx.class, "gripServo");
        linerRailServo.initServo("linearRailServo", getOpModeEX().hardwareMap);
        nest.initServo("nest", getOpModeEX().hardwareMap);
        linearPosition.init(getOpModeEX().hardwareMap, "axon2");

        clawSensor = getOpModeEX().hardwareMap.get(TouchSensor.class, "clawIR");
        nestSensor = getOpModeEX().hardwareMap.get(TouchSensor.class, "nestIR");

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(335);
        linerRailServo.setRange(1800);
        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 270);
        gripServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        nest.setRange(new PwmControl.PwmRange(500, 2500), 270);

        slidesReset = getOpModeEX().hardwareMap.get(TouchSensor.class, "CollectionReset");

        closeAim = getOpModeEX().hardwareMap.get(WebcamName.class, "webcam");
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(closeAim);
        builder.addProcessor(sampleSorterContour);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(1280, 960));
        portal = builder.build();

        profile.isVertical(false);

        fourBarMainPivot.setDirection(Servo.Direction.REVERSE);
        fourBarMainPivot.setOffset(36);

//        fourBarMainPivot.setPosition(90);
//        fourBarSecondPivot.setPosition(100);

        nest.setDirection(Servo.Direction.REVERSE);
        nest.setOffset(5);
        nest.setPosition(135);

        griperRotate.setDirection(Servo.Direction.REVERSE);
        griperRotate.setOffset(-10);
        griperRotate.setPosition(90);

        gripServo.setPosition(0);

        Stowed.execute();

        //max left = 570
        //max right = 1340
        linerRailServo.setDirection(Servo.Direction.REVERSE);
        linerRailServo.setPosition(900);

    }

    @Override
    public void execute() {

        executeEX();

        if (fourBarState == fourBar.collect && !clawSensor.isPressed() && !autoCollected && getCurrentCommand() != transfer){
            if (getChamberCollect()){
                queueCommand(chamberCollect);
            }else {
                queueCommand(transfer);
            }

            autoCollected = true;
        } else if (autoCollected && fourBarState == fourBar.transferUp) {
            autoCollected = false;
        }

        double ticksPerCM = (double) 440 / 35;
        double error = Math.abs((slideTarget*ticksPerCM) - (double) horizontalMotor.getCurrentPosition());

        System.out.println("error PID" + error);

        if (slidesReset.isPressed() && slideTarget == 0 && !resettingSlides){
            extendoPower = 0;
        }else if (error > 3 && !resettingSlides){

            if((slideTarget*ticksPerCM) > horizontalMotor.getCurrentPosition()){
                extendoPower = adjustment.calculate((slideTarget*ticksPerCM) ,  horizontalMotor.getCurrentPosition())+0.1;
            } else {
                extendoPower = adjustment.calculate((slideTarget*ticksPerCM) ,  horizontalMotor.getCurrentPosition())-0.1;
            }

            if (horizontalMotor.getVelocity() < 5 && error > 15){
                slideI += 0.00008;
            }else {
                slideI = 0;
            }

            adjustment.setI(slideI);

            System.out.println("extendoPower PID" + extendoPower);
        }else {
            slideI = 0;
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

        if (nestState == Nest.sample){
            nest.setPosition(135);
        } else if (nestState == Nest.specimen) {
            nest.setPosition(45);
        }

        if (clawsState == clawState.grab){
            gripServo.setPosition(0);
        } else if (clawsState == clawState.drop) {
            gripServo.setPosition(0.45);
        } else if (clawsState == clawState.slightRelease) {
            gripServo.setPosition(0.09);
        }

        try {
            fWriter.write("Rail position " + getRailPosition());
            fWriter.write(System.lineSeparator());
            fWriter.write("Slides position " +  horizontalMotor.getCurrentPosition()/(440/35));
            fWriter.write(System.lineSeparator());
            fWriter.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        horizontalMotor.update(extendoPower);
        horizontalMotor.updateVelocity();
    }

    private final Command preCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPreCollect);
                fourBarSecondPivot.setPosition(secondPivotPreCollect);
//                griperRotate.setPosition(rotatePreCollect);
                clawsState = clawState.drop;
            }
    );

    private final Command Collect = new Execute(
            () -> {
                System.out.println("Running collect command");

                fourBarMainPivot.setPosition(mainPivotCollect);
                fourBarSecondPivot.setPosition(secondPivotCollect);
            }
    );

    private final Command transferUp = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotTransferUp);
                fourBarSecondPivot.setPosition(secondPivotTransferUp);
            }
    );

    private final Command AboveNest = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotAboveNest);
                fourBarSecondPivot.setPosition(secondPivotAboveNest);
            }
    );

    private final Command Stowed = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotStow);
                fourBarSecondPivot.setPosition(secondPivotStow);
            }
    );

    private final Command dropNest = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPlaceNest);
                fourBarSecondPivot.setPosition(secondPlaceNest);
                griperRotate.setPosition(rotatePlaceNest);
            }
    );

    public final Command pullOut = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotDropNestPull);
                fourBarSecondPivot.setPosition(secondDropNestPull);
                griperRotate.setPosition(rotateDropNestPull);
            }
    );

    private final Command transInt = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotTransInt);
                fourBarSecondPivot.setPosition(secondPivotTransInt);
                griperRotate.setPosition(rotateTransInt);
//                setClawsState(clawState.slightRelease);
            }
    );

    public final Command camera = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotCamera);
                fourBarSecondPivot.setPosition(secondPivotCamera);
                griperRotate.setPosition(rotatePreCollect);
            }
    );

    public final Command ChamberCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotChamberCollect);
                fourBarSecondPivot.setPosition(secondPivotChamberCollect);
                griperRotate.setPosition(rotateChamberCollect);
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

    public Command cameraSetPoint(double slideTarget){
        this.slideTarget = slideTarget;
        return camera;
    }

    public Command collect = new LambdaCommand(
            () -> {},
            () -> {

                System.out.println("Running collect command FULL");

                setNestState(Nest.sample);

                if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    Collect.execute();

                } else if (fourBarState == fourBar.collect) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;

                    preCollect.execute();

                }else if (!(fourBarState == fourBar.transferringStates)){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;

                    preCollect.execute();

                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

            },
            () -> !(fourBarState == fourBar.transferringStates) && fourBarTimer.milliseconds() > transferWaitTime
    );

    public Command autoCollect = new LambdaCommand(
            () -> {
                camera.execute();
                setRailTargetPosition(9);
                autoCollectTimer.reset();
                targetingState = targeting.transferring;
                targetTargetingState = targeting.camera;
                cameraWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCamera)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotCamera)*(microRoboticTime+2));
            },
            () -> {

                setNestState(Nest.sample);

                if (targetingState == targeting.camera){

                    System.out.println("First Detect");

                    autoCollectTimer.reset();
                    targetingState = targeting.transferring;
                    targetTargetingState = targeting.detecting;
                    cameraWaitTime = 1000;

                    sampleSorter.resetDetectionCounter();
//                    clawsState = clawState.grab;

                }else if (targetingState == targeting.detecting){
                    System.out.println("First Movement");

                    autoCollectTimer.reset();
                    targetingState = targeting.transferring;
                    targetTargetingState = targeting.movingToTargeting;
                    cameraWaitTime = 1000;

                    if (sampleSorter.getRailTarget() < 0){
                        setRailTargetPosition(0);
                    }else if (sampleSorter.getRailTarget() > 20){
                        setRailTargetPosition(20);
                    }else {
                        setRailTargetPosition(sampleSorter.getRailTarget()-1);
                    }

                    double newSlidesTarget = slideTarget + sampleSorter.getSlidesDelta() + 5;

                    if (newSlidesTarget < 0){
//                        profile.generateMotionProfile(0, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;
                        this.slideTarget = 0;
                    }else if (newSlidesTarget > 35){
//                        profile.generateMotionProfile(35, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;

                        this.slideTarget = 35;
                    }else {
//                        profile.generateMotionProfile(newSlidesTarget, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;
                        this.slideTarget = newSlidesTarget;
                    }

                    griperRotate.setPosition(90+sampleSorter.getAngleRotate());

                }else if (targetingState == targeting.movingToTargeting){

                    System.out.println("Second Detect");

                    autoCollectTimer.reset();
                    cameraWaitTime = 1000;
                    sampleSorter.resetAve();
                    targetingState = targeting.transferring;
                    targetTargetingState = targeting.targeting;

                    sampleSorter.resetDetectionCounter();

                } else if (targetingState == targeting.targeting){

                    System.out.println("Second Movement");

                    autoCollectTimer.reset();
//                        cameraWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    cameraWaitTime = 500;
                    targetingState = targeting.transferring;
                    targetTargetingState = targeting.collecting;

                    double railTarget = sampleSorter.getRailTarget()-11;
                    railTarget = getRailPosition()+railTarget;

                    if (railTarget < 0){
                        setRailTargetPosition(0);
                    }else if (railTarget > 20){
                        setRailTargetPosition(20);
                    }else {
                        setRailTargetPosition(railTarget);
                    }

                    double newSlidesTarget = slideTarget + sampleSorter.getSlidesDelta();

                    if (newSlidesTarget < 0){
//                        profile.generateMotionProfile(0, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;
                        this.slideTarget = 0;
                    }else if (newSlidesTarget > 35){
//                        profile.generateMotionProfile(35, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;
                        this.slideTarget = 35;
                    }else {
//                        profile.generateMotionProfile(newSlidesTarget, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;
                        this.slideTarget = newSlidesTarget;
                    }

//                    if (newSlidesTarget > 0 && newSlidesTarget < 35){
//                        if (collection.sampleSorter.getSlidesDelta() > 0){
//                            collection.queueCommand(collection.collect(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + collection.sampleSorter.getSlidesDelta())));
//                        }else {
//                            collection.queueCommand(collection.collect(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + collection.sampleSorter.getSlidesDelta())));
//                        }
////                collection.queueCommand(collection.collect);
//                    }

                    griperRotate.setPosition(90+sampleSorter.getAngleRotate());

                    preCollect.execute();
                    fourBarState = fourBar.preCollect;
                    rotateWhileCollecting = sampleSorter.isRotate();
                    queueCommand(collect);
                    rotateWhileCollecting = true;
//                    autoCollecting = true;

                }

                if (targetingState == targeting.transferring && autoCollectTimer.milliseconds() > cameraWaitTime && targetTargetingState != targeting.detecting){
                    targetingState = targetTargetingState;
                }else if (targetingState == targeting.transferring && sampleSorter.getDetectionCounter() >= 2 && (targetTargetingState == targeting.detecting || targetTargetingState == targeting.movingToTargeting)){
                    targetingState = targetTargetingState;
                }

            },
            () -> targetingState == targeting.collecting
    );

    public Command autoCollectSingleDetect = new LambdaCommand(
            () -> {
                camera.execute();
                setRailTargetPosition(9);
                autoCollectTimer.reset();
                targetingState = targeting.transferring;
                targetTargetingState = targeting.camera;
                cameraWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCamera)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotCamera)*(microRoboticTime+2));
            },
            () -> {

                setNestState(Nest.sample);

                if (targetingState == targeting.camera){

                    autoCollectTimer.reset();
                    targetingState = targeting.transferring;
                    targetTargetingState = targeting.targeting;
                    cameraWaitTime = 1000;

                    sampleSorter.resetDetectionCounter();
//                    clawsState = clawState.grab;

                }else if (targetingState == targeting.targeting){

                    autoCollectTimer.reset();
//                        cameraWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    cameraWaitTime = 1000;
                    targetingState = targeting.transferring;
                    targetTargetingState = targeting.collecting;

                    double railTarget = sampleSorter.getRailTarget()-10;
                    railTarget = getRailPosition()+railTarget;

                    if (railTarget < 0){
                        setRailTargetPosition(0);
                    }else if (railTarget > 20){
                        setRailTargetPosition(20);
                    }else {
                        setRailTargetPosition(railTarget);
                    }

                    double newSlidesTarget = horizontalMotor.getCurrentPosition() / ((double) 440 /35) + sampleSorter.getSlidesDelta();

                    if (newSlidesTarget < 0){
//                        profile.generateMotionProfile(0, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;
                        this.slideTarget = 0;
                    }else if (newSlidesTarget > 35){
//                        profile.generateMotionProfile(35, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;
                        this.slideTarget = 35;
                    }else {
//                        profile.generateMotionProfile(newSlidesTarget, horizontalMotor.getCurrentPosition());
//                        slidesState = slideState.profile;
                        this.slideTarget = newSlidesTarget;
                    }

//                    if (newSlidesTarget > 0 && newSlidesTarget < 35){
//                        if (collection.sampleSorter.getSlidesDelta() > 0){
//                            collection.queueCommand(collection.collect(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + collection.sampleSorter.getSlidesDelta())));
//                        }else {
//                            collection.queueCommand(collection.collect(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + collection.sampleSorter.getSlidesDelta())));
//                        }
////                collection.queueCommand(collection.collect);
//                    }

                    griperRotate.setPosition(90+sampleSorter.getAngleRotate());

                    preCollect.execute();
                    fourBarState = fourBar.preCollect;
                    rotateWhileCollecting = sampleSorter.isRotate();
                    queueCommand(collect);
//                    autoCollecting = true;

                }

                if (targetingState == targeting.transferring && autoCollectTimer.milliseconds() > cameraWaitTime && targetTargetingState != targeting.detecting){
                    targetingState = targetTargetingState;
                }else if (targetingState == targeting.transferring && sampleSorter.getDetectionCounter() >= 4 && (targetTargetingState == targeting.detecting || targetTargetingState == targeting.targeting)){
                    targetingState = targetTargetingState;
                }

            },
            () -> targetingState == targeting.collecting
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
                preCollect.execute();
                setRailTargetPosition(9);
                autoCollectTimer.reset();
                targetingState = targeting.transferring;
                targetTargetingState = targeting.camera;
                cameraWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCamera)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotCamera)*(microRoboticTime+2));

                sampleSorterContour.setScanning(false);
            },
            () -> {

                setNestState(Nest.sample);

//                Point targetPointGlobal = new Point(141, 100);

                double angle = findAngle(targetPointGlobal, new Point(RobotPosition.getVertical(), RobotPosition.getHorizontal()));

                double xError = targetPointGlobal.x-RobotPosition.getVertical();
                double yError = RobotPosition.getHorizontal()-targetPointGlobal.y;

                double angleDelta = Math.abs(angle - RobotPosition.getPivot());

                double deltaHeading = Math.toRadians(RobotPosition.getPivot() - angle);

                double convertedDelta = Math.toDegrees(deltaHeading);

                if (convertedDelta < -180) {
                    deltaHeading = Math.toRadians(-(convertedDelta + 360));
                } else if (convertedDelta > 180) {
                    deltaHeading = Math.toRadians(360 - convertedDelta);
                }

                double disToTarget = Math.hypot(xError, yError);

                double railDisToTarget;
                double v = disToTarget * Math.sin((deltaHeading));

                if (Math.toDegrees(deltaHeading) > 0){
                    railDisToTarget = v;
                }else {
                    railDisToTarget = -v;
                }

                double otherAngle = (180 - angleDelta)/2;
                double targetRailPosition;
                double slideTarget;

                if (deltaHeading == 0){
                    targetRailPosition = 9.5;
                    slideTarget = disToTarget-32;
                }else {
                    targetRailPosition = 9.5 + (railDisToTarget * Math.sin(Math.toRadians(otherAngle)));
                    slideTarget = (disToTarget - (Math.abs((railDisToTarget * Math.cos((Math.toRadians(otherAngle)))))))-32;
//                    railTarget = xError;
//                    slideTargetPosition = slideTarget;
                }

                railTarget = targetRailPosition;
                slideTargetPosition = slideTarget;

                try {
                    fWriter.write("xError " + xError);
                    fWriter.write(System.lineSeparator());
                    fWriter.write("yError " + yError);
                    fWriter.write(System.lineSeparator());
                    fWriter.write("targetRailPosition " + targetRailPosition);
                    fWriter.write(System.lineSeparator());
                    fWriter.write("slideTarget " + slideTarget);
                    fWriter.write(System.lineSeparator());

                    fWriter.flush();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }

                if (slideTarget > 36 || targetRailPosition > 20 || targetRailPosition < 0){
                    System.out.println("Out of range");
                    targetingState = targeting.collecting;
                }else {
                    setSlideTarget(slideTarget);
                    setRailTargetPosition(targetRailPosition);
                    griperRotate.setPosition(90+Math.toDegrees(deltaHeading));
                }

            },
            () -> targetingState == targeting.collecting
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

                if (autoCollecting && fourBarState == fourBar.collect && clawsState == clawState.grab){

                    clawsState = clawState.drop;

                    fourBarTimer.reset();
                    transferWaitTime = 1000;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    fourBarMainPivot.setPosition(102);
                    autoCollecting = false;

                } else if (fourBarState == fourBar.collect && clawsState == clawState.drop){

                    clawsState = clawState.grab;

                    fourBarMainPivot.setPosition(92);

                    fourBarTimer.reset();
                    transferWaitTime = 600;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                } else if (fourBarState == fourBar.collect && clawsState == clawState.grab){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransInt)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    transInt.execute();
                    setRailTargetPosition(railTargetTransInt);

                } else if (fourBarState == fourBar.transferInt && !clawSensor.isPressed()) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    slidesState = slideState.profile;
                    setSlideTarget(0);
                    transferUp.execute();
                    setClawsState(clawState.grab);

                } else if (fourBarState == fourBar.transferInt && clawSensor.isPressed()) {

                    cancelTransfer = true;
                    queueCommand(collect);

                } else if (fourBarState == fourBar.transferUp && griperRotate.getPositionDegrees() > 160) {

                    fourBarTimer.reset();
                    transferWaitTime = 500;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    griperRotate.setPosition(rotatePlaceNest);

                }else if (fourBarState == fourBar.transferUp && griperRotate.getPositionDegrees() < 100) {

                    fourBarTimer.reset();
//                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotAboveNest)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotAboveNest)*microRoboticTime);
                    transferWaitTime = 400;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.aboveNest;
//
//                    fourBarSecondPivot.setPosition(170);

                    clawsState = clawState.slightRelease;
                    fourBarMainPivot.setPosition(mainPivotAboveNest);
                    fourBarSecondPivot.setPosition(secondPivotAboveNest);

                }else if (fourBarState == fourBar.aboveNest) {

                    fourBarTimer.reset();
//                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotPlaceNest)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPlaceNest)*microRoboticTime);
                    transferWaitTime = 200;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.dropNest;

                    dropNest.execute();

                }else if (fourBarState == fourBar.dropNest && (clawsState == clawState.grab || clawsState == clawState.slightRelease)) {

                    clawsState = clawState.drop;

                    fourBarTimer.reset();
                    transferWaitTime = 400;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.dropNest;

                    dropNest.execute();

                }else if (fourBarState == fourBar.dropNest && clawsState == clawState.drop) {

                    fourBarTimer.reset();
                    transferWaitTime = 90;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    Stowed.execute();

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }
//                else if (fourBarState == fourBar.dropNest && clawsState == clawState.drop) {
//
//                    fourBarTimer.reset();
//                    transferWaitTime = 350;
//                    fourBarState = fourBar.transferringStates;
//                    fourBarTargetState = fourBar.dropPullOut;
//
//                    pullOut.execute();
//
//                }


            },
            () -> (fourBarState == fourBar.transferUp && clawsState == clawState.drop) || cancelTransfer
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
                    transferWaitTime = 500;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collectChamber;

                    ChamberCollect.execute();

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.collectChamber
    );

    public Command stow = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.collect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    transInt.execute();

                }else if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    transInt.execute();

                }else if (fourBarState == fourBar.collectChamber){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    transInt.execute();

                }else if (fourBarState == fourBar.dropNest) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                }else if (fourBarState == fourBar.transferInt) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    transferUp.execute();

                } else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                    if (fourBarState == fourBar.transferUp){
                        griperRotate.setPosition(rotatePlaceNest);
                    }

                }

            },
            () -> fourBarState == fourBar.transferUp
    );

    public double getRailPosition() {
        double degreesPerCM = (double) 720 / 20;
        return (linerRailServo.getPositionDegrees() - 570)/degreesPerCM;
    }

    public void setTargetPoint(Point targetPoint){
        this.targetPointGlobal = targetPoint;
    }

    public void setRailTargetPosition(double targetPosition) {
        this.railTargetPosition = targetPosition;
        double degreesPerCM = (double) 720 /20;

        double servoTarget = 570+(degreesPerCM*targetPosition);

        if (servoTarget < 570){
            linerRailServo.setPosition(570);
        } else if (servoTarget > 1340) {
            linerRailServo.setPosition(1340);
        }else {
            linerRailServo.setPosition(servoTarget);
        }

    }


    private static double findRealDelta(double last, double current){
        double realDelta;

        if (last > current){
            realDelta = current + (360 - last);
        }else {
            realDelta = last + (360 - current);
        }

        return realDelta;
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
        gripServo.getController().pwmDisable();
        linerRailServo.disableServo();
        nest.disableServo();
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

}
