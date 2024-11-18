package dev.weaponboy.command_library.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import dev.weaponboy.vision.Testing_SIM.PerspectiveTransformPipeline;
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
    public ServoImplEx gripServo;
    public ServoDegrees linerRailServo=new ServoDegrees();
    public ServoDegrees nest = new ServoDegrees();

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

    boolean TransferDrop =false;
    ElapsedTime WaitForTranferDrop = new ElapsedTime();

    /**states*/
    public enum fourBar{
        preCollect,
        collect,
        detectingSample,
        transferringStates,
        collectChamber,
        transferUp,
        stowed,
        transferInt,
        transfering,
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
    //16/40

    /**
     * collect position values
     * */
    double mainPivotCollect = 77;
    double secondPivotCollect = 320;

    /**
     * preCollect position values
     * */
    double mainPivotPreCollect = 105;
    double secondPivotPreCollect = 310;
    double rotatePreCollect = 90;

    /**
     * stowed position values
     * */
    double mainPivotStow = 180;
    double secondPivotStow = 180;

    /**
     * stowed position values
     * */
    double rotateTransInt = 90;
    double railTargetTransInt = 10;
    double mainPivotTransInt = 180;
    double secondPivotTransInt = 180;



    /**
     * stowed position values
     * */
    double mainPivotTransferUp = 210;
    double secondPivotTransferUp = 100;
    /**
     * stowed position values
     * */
    double mainPivotTransfer = 210;
    double secondPivotTransfer = 122;

    /**
     * stow position values
     * */
    double mainPivotChamberCollect = 120;
    double secondPivotChamberCollect = 310;
    double rotateChamberCollect = 90;



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
        gripServo = getOpModeEX().hardwareMap.get(ServoImplEx.class, "gripServo");
        linerRailServo.initServo("linearRailServo", getOpModeEX().hardwareMap);
        nest.initServo("nest", getOpModeEX().hardwareMap);
        linearPosition.init(getOpModeEX().hardwareMap, "axon2");

        clawSensor = getOpModeEX().hardwareMap.get(TouchSensor.class, "clawIR");

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

        fourBarMainPivot.setOffset(10);

        fourBarSecondPivot.setOffset(-20);



        nest.setDirection(Servo.Direction.REVERSE);
        nest.setOffset(5);
        nest.setPosition(135);

        griperRotate.setDirection(Servo.Direction.REVERSE);
        griperRotate.setOffset(-10);
        griperRotate.setPosition(90);

        setClawsState(clawState.drop);


        Stowed.execute();

        //max left = 570
        //max right = 1340
        linerRailServo.setDirection(Servo.Direction.REVERSE);
        setRailTargetPosition(railTargetTransInt);
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

        double ticksPerCM = (double) 492 / 52;
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

//            adjustment.setI(slideI);

            System.out.println("extendoPower PID" + extendoPower);
        }else {
            slideI = 0;
        }

        if (!resettingSlides && !slidesReset.isPressed() && slideTarget == 0 && Math.abs(horizontalMotor.getVelocity()) < 10){
            slideTarget = 0;
            extendoPower = -0.4;
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
            gripServo.setPosition(0.3);
        } else if (clawsState == clawState.drop) {
            gripServo.setPosition(0.70);
        } else if (clawsState == clawState.slightRelease) {
            gripServo.setPosition(0.4);
        }

        horizontalMotor.update(extendoPower);
        horizontalMotor.updateVelocity();
    }

    private final Command preCollect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotPreCollect);
                fourBarSecondPivot.setPosition(secondPivotPreCollect);
                clawsState = clawState.drop;
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



    private final Command Stowed = new Execute(
            () -> {
                fourBarMainPivot.setPosition(mainPivotStow);
                fourBarSecondPivot.setPosition(secondPivotStow);
            }
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
            () -> TransferDrop && WaitForTranferDrop.milliseconds()>100

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

                if (sampleSorterContour.isScanning() && !sampleSorterContour.detections.isEmpty()){
                    transferWaitTime = 10;
                    sampleSorterContour.setScanning(false);
                }else if (!sampleSorterContour.isScanning()) {
                    transferWaitTime = 500;
                    sampleSorterContour.setScanning(true);
                    detectionAttemptsCounter++;
                }

            },
            () -> {

                if (fourBarState == fourBar.detectingSample){

                    sampleSorterContour.setScanning(false);

                    if (!sampleSorterContour.detections.isEmpty()){

                        sampleMap = sampleSorterContour.convertPositionsToFieldPositions(RobotPosition);
                        angle = sampleMap.get(0).getAngle();

                        preCollect.execute();

                        fourBarTimer.reset();
                        fourBarState = fourBar.transferringStates;
                        fourBarTargetState = fourBar.preCollect;
                        transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotPreCollect)*(microRoboticTime), Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*microRoboticTime, Math.abs(slideTarget - horizontalMotor.getCurrentPosition())*5));

                        setNestState(Nest.sample);

                        targetPointGlobal = sampleMap.get(0).getTargetPoint();

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
                            slideTarget = disToTarget-36;
                        }else {
                            targetRailPosition = 9.5 + (railDisToTarget * Math.sin(Math.toRadians(otherAngle)));

                            slideTarget = (disToTarget - Math.abs((railDisToTarget * Math.cos((Math.toRadians(otherAngle))))))-31;
                        }

                        railTarget = (Math.abs((railDisToTarget * Math.cos((Math.toRadians(otherAngle))))));
                        slideTargetPosition = slideTarget;

                        if (slideTarget > 36 || targetRailPosition > 20 || targetRailPosition < 0){
                            System.out.println("Out of range");
                            targetingState = targeting.collecting;
                        }else {
                            setSlideTarget(slideTarget);
                            setRailTargetPosition(targetRailPosition);
                            griperRotate.setPosition((180+Math.toDegrees(deltaHeading)) - sampleMap.get(0).getAngle());

                            FileWriter fWriter = null;
                            try {
                                fWriter = new FileWriter("/sdcard/VisionLogs.txt", true);

                                fWriter.write(System.lineSeparator());
                                fWriter.write(System.lineSeparator());
                                fWriter.write("current point " + new Point(RobotPosition.getVertical(), RobotPosition.getHorizontal()));
                                fWriter.write(System.lineSeparator());
                                fWriter.write("target point " + targetPointGlobal);
                                fWriter.write(System.lineSeparator());
                                fWriter.write("xError " + xError);
                                fWriter.write(System.lineSeparator());
                                fWriter.write("yError " + yError);
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

                        if (detectionAttemptsCounter <= 2){
                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.detectingSample;

                            if (sampleSorterContour.isScanning() && !sampleSorterContour.detections.isEmpty()){
                                transferWaitTime = 10;
                                sampleSorterContour.setScanning(false);
                            }else if (!sampleSorterContour.isScanning()) {
                                transferWaitTime = 500;
                                detectionAttemptsCounter++;
                                sampleSorterContour.setScanning(true);
                            }
                        }else {
                            Stowed.execute();

                            fourBarTimer.reset();
                            fourBarState = fourBar.transferringStates;
                            fourBarTargetState = fourBar.stowed;
                            transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*(microRoboticTime), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);

                        }


                    }

                }else if (fourBarState == fourBar.preCollect){

                    Collect.execute();

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCollect)*(microRoboticTime+10), Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*microRoboticTime);

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.collect || (fourBarState == fourBar.stowed && fourBarTimer.milliseconds() > transferWaitTime)
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

                if ( fourBarState == fourBar.collect && clawsState == clawState.drop){

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = 1000;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    autoCollecting = false;

                } else if (fourBarState == fourBar.collect && clawsState == clawState.grab){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransInt)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    transInt.execute();
                    setRailTargetPosition(railTargetTransInt);

                } else if (fourBarState == fourBar.transferInt ) {
                    setSlideTarget(0);
                    setClawsState(clawState.grab);
                    fourBarTimer.reset();
                    Transfer.execute();
                    fourBarState = fourBar.transfering;

                }
                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }
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

                }else if (fourBarState == fourBar.transfering) {

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
