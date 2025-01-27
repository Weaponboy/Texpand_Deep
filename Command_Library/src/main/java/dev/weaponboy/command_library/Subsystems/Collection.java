package dev.weaponboy.command_library.Subsystems;

import android.util.SparseArray;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.ArrayList;

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
import dev.weaponboy.vision.detectionData;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;

public class Collection extends SubSystem {

    public ArrayList<detectionData> sampleMap = new ArrayList<>();

    public findAngleUsingContour sampleDetector = new findAngleUsingContour();
    public VisionPortal portal;

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

    public boolean getChamberCollect() {
        return chamberCollectBool;
    }

    public void setChamberCollect(boolean chamberCollectBool) {
        this.chamberCollectBool = chamberCollectBool;
    }

    boolean chamberCollectBool = false;

    boolean runSet = false;
    boolean TransferDrop = false;
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

    public enum clawState{
        drop,
        grab,
        slightRelease,
        openFull
    }

    /**
     * collect position values
     * */
    double mainPivotCollect = 86;
    double secondPivotCollect = 317;


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
     * collect position values
     * */
    double mainPivotSpikePush = 65;
    double secondPivotSpikePush = 325;

    /**
     * preCollect position values
     * */
    double mainPivotMidTransfer = 180;
    double secondPivotMidTransfer = 160;

    /**
     * preCollect position values
     * */
    double mainPivotWallCollect = 125;
    double secondPivotWallCollect = 245;

    /**
     * low chamber position values
     * */
    double mainPivotLowChamberPreClip = 135;
    double secondPivotLowChamberPreClip = 220;

    /**
     * low chamber position values
     * */
    double mainPivotLowChamberClip = 105;
    double secondPivotLowChamberClip = 240;

    /**
     * stowed position values
     * */
    double mainPivotStow = 165;
    double secondPivotStow = 152;
    double turretTransferPosition = 167.5;

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
    double mainPivotTransferSlam = 180;
    double secondPivotTransferSlam = 144;

    /**
     * stowed position values
     * */
    double mainPivotTransferAuto = 180;
    double secondPivotTransferAuto = 144;

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

    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;

    boolean autoCollecting = false;

    /**enum states*/
    private fourBar fourBarState = fourBar.stowed;
    private fourBar fourBarTargetState = fourBar.stowed;
    private clawState clawsState = clawState.drop;

    /**PID controllers**/
    PIDController adjustment = new PIDController(0.015, 0, 0.05);
    PIDController adjustmentClose = new PIDController(0.009, 0, 0.009);

    /**Slide target stuff**/
    double slideTarget;
    boolean longTarget = false;

    boolean transferCanceled = false;
    boolean cancelTransfer = true;
    int transferCounter = 0;

    public Vector2D targetPosition = new Vector2D(clawOffsetFromSlides, clawOffsetFromSlides);
    public Vector2D targetPositionManuel = new Vector2D(clawOffsetFromSlides, clawOffsetFromSlides);

    /**gripper positions*/
    double gripperDrop = 122;
    double gripperGrab = 43;
    double gripperSlightRelease = 70;
    double gripperOpenFull = 122;

    boolean braking = false;
    ElapsedTime brakingTimer = new ElapsedTime();

    public double slideTargetPosition;
    public double railTarget;

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

    @Override
    public void execute() {

        executeEX();

        double ticksPerCM = (double) 205 / 18;
        double error;

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
        }

        if (isTransferCanceled() && fourBarState != fourBar.preCollect){
            transferCanceled = false;
        }

        horizontalMotor.update(Range.clip(extendoPower, -1, 0.8));

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
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotCollect)*2, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotPreCollect)*2);

                    Collect.execute();

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
                System.out.println("SLIDE TARGET" + newSlideTarget);
                System.out.println("SLIDE TARGET" + getSlideTarget());

                if (newSlideTarget != 18763){
                    setSlideTarget(newSlideTarget);
                    System.out.println("RAN SET IN EXTENDO TARGETING" + runSet);
                    runSet = true;
                }

                System.out.println("Condition 1" + (runSet && Math.abs(getSlideTarget() - getSlidePositionCM()) < 1.1 && Math.abs(horizontalMotor.getVelocity()) < 10 && Math.abs(extendoPower) < 0.1 && getSlideTarget() > 1));
                System.out.println("Condition 2" + (!runSet && abortTimer.milliseconds() > abortTime));

//                }
            },
            () -> runSet && Math.abs(getSlideTarget() - getSlidePositionCM()) < 1.1 && Math.abs(horizontalMotor.getVelocity()) < 10 && Math.abs(extendoPower) < 0.1 && getSlideTarget() > 1 || !runSet && abortTimer.milliseconds() > abortTime
    );

    public Command autoCollectGlobal(TargetSample targetPoint){
        System.out.println("Running the global collect" + targetPoint.getTargetPoint().getX() + " : " + targetPoint.getTargetPoint().getY());
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

                    sampleDetector.setScanning(false);

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
                            turret.setPosition(turretTransferPosition);

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

    public void targetPointWithExtendo(Vector2D targetPoint){

        targetPosition = targetPoint;

        calculateKinematicsGlobal();

    }

    public void targetPointWithExtendoNoArm(Vector2D targetPoint){

        targetPosition = targetPoint;

        calculateKinematicsGlobal();
    }

    public Command transfer = new LambdaCommand(
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

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*6, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransfer)*4);
                    fourBarTargetState = fourBar.transferUp;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {
                        setSlideTarget(0);
                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        midTransfer.execute();

//                        if (horizontalMotor.getCurrentPosition() < 320){
//
////                            setClawsState(clawState.slightRelease);
////                            gripperReleaseTimer.reset();
//
//                            commandTimer.reset();
//                        }else{
//                            fourBarMainPivot.setPosition(mainPivotCollect+20);
//                            transferToFar = true;
//                        }

                    }


                }

//                else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab && (getRailPosition() > 16 || getRailPosition() < 10 || griperRotate.getPositionDegrees() < 100)){
//
//                    fourBarTimer.reset();
//
//                    fourBarState = fourBar.transferringStates;
//                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*5, Math.max(Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*microRoboticTime, Math.abs(getRailPosition() - railTargetTransInt)*8));
//                    fourBarTargetState = fourBar.collect;
//
//                    fourBarMainPivot.setPosition(mainPivotPreCollect+20);
//                    fourBarSecondPivot.setPosition(secondPivotPreCollect);
////                    setClawsState(clawState.grab);
//
//                    griperRotate.setPosition(rotateTransfer);
//                    setRailTargetPosition(railTargetTransInt);
//
//                }


//                if (releasingABit && gripperReleaseTimer.milliseconds() > 100){
//                    setClawsState(clawState.grab);
//                }
//
//                if (horizontalMotor.getCurrentPosition() < 320 && transferToFar){
//                    Transfer.execute();
//                    transferToFar = false;
//                }

                if (clawsState == clawState.grab && fourBarTargetState != fourBar.collect){
                    transferCounter++;
                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

                if(isCancelTransferActive() && !breakBeam.isPressed() && clawsState == clawState.grab && fourBarTargetState != fourBar.collect && transferCounter < 5){
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
            },
            () -> {

                if (!cancelTransfer && fourBarState == fourBar.collect && (clawsState == clawState.drop || clawsState == clawState.openFull) && horizontalMotor.getVelocity() < 5) {

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab && (griperRotate.getPositionDegrees() < 90)){

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*3, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*microRoboticTime);
                    fourBarTargetState = fourBar.collect;

                    fourBarMainPivot.setPosition(mainPivotPreCollect+20);
                    fourBarSecondPivot.setPosition(secondPivotPreCollect);
//                    setClawsState(clawState.grab);

                    griperRotate.setPosition(rotateTransfer);
                    turret.setPosition(turretTransferPosition);

                }else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab) {

                    fourBarTimer.reset();
                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*3, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransferSlam)*4);
                    fourBarTargetState = fourBar.transferUp;

                    if(isCancelTransferActive() && !breakBeam.isPressed()){

                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        setSlideTarget(0);

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

                if (clawsState == clawState.grab && fourBarTargetState != fourBar.collect){
                    transferCounter++;
                }

                if (fourBarState == fourBar.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourBarState = fourBarTargetState;
                }

                if(isCancelTransferActive() && !breakBeam.isPressed() && clawsState == clawState.grab && fourBarTargetState != fourBar.collect && transferCounter < 7){
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

                } else if (!cancelTransfer && fourBarState == fourBar.collect && clawsState == clawState.grab && (griperRotate.getPositionDegrees() < 100)){

                    fourBarTimer.reset();

                    fourBarState = fourBar.transferringStates;
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransfer)*1, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotMidTransfer)*1);
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

                    }else {

                        setClawsState(clawState.grab);

                        griperRotate.setPosition(rotateTransfer);
                        turret.setPosition(turretTransferPosition);

                        setSlideTarget(0);

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

//                if (clawsState == clawState.grab && fourBarTargetState != fourBar.collect){
//                    transferCounter++;
//                }

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

                System.out.println("cancel transfer: " + cancelTransfer);

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
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransfer)*microRoboticTime);
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

        if (slideTarget > 52){
            this.slideTarget = 52;
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

        System.out.println("RobotPosition.getPivot(): " + RobotPosition.getPivot());

        System.out.println("errors.gety(): " + targetPosition.getY());
        System.out.println("errors.getx(): " + (targetPosition.getX()));

        System.out.println("errors.getY(): " + errors.getY());
        System.out.println("errors.getX(): " + errors.getX());

        targetPositionManuel = new Vector2D(errors.getX() - robotLength, clawOffsetFromSlides + errors.getY());

        double slideOffset = 0;

        if (errors.getY() < -20 || errors.getY() > 20){
//            clearQueue();
            return 18763;
        }else{

            double angle = Math.toDegrees(Math.acos(errors.getY() / clawOffsetFromSlides));

            double hypotSquared = (clawOffsetFromSlides * clawOffsetFromSlides) - (Math.abs(errors.getY()) * Math.abs(errors.getY()));

            slideOffset = Math.sqrt(hypotSquared);

            double returnTarget = (((errors.getX()) - robotLength) - slideOffset)+2;

            if (returnTarget < 62){

                turret.setPosition(77.5 + angle);

                if (errors.getY() < -6){
                    parallelAngle = 0 + (turret.getPositionDegrees() - turretTransferPosition);
                }else {
                    parallelAngle = 180 + (turret.getPositionDegrees() - turretTransferPosition);
                }

                if (this.angle > 75 || this.angle < -75){
                    griperRotate.setPosition(parallelAngle);
                }else{
                    double perAngle = 0;

                    if (parallelAngle > 90){
                        perAngle = parallelAngle - 90;
                    }else if (parallelAngle < 90){
                        perAngle = parallelAngle + 90;
                    }

                    double realAngle = perAngle - this.angle;

                    if (realAngle > 180){
                        realAngle = realAngle - 180;
                    } else if (realAngle < 0) {
                        realAngle = realAngle + 180;
                    }

                    griperRotate.setPosition(realAngle);
                }

                targetPositionManuel = new Vector2D(errors.getX() - robotLength, clawOffsetFromSlides - errors.getY());
                return returnTarget;

            }else {
//                clearQueue();
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

    public double getSlideTarget() {
        return slideTarget;
    }

    public void updateRobotPosition(RobotPower robotPosition){
        RobotPosition = robotPosition;
    }

}
