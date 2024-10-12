package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class Collection2 extends SubSystem {

    // slides
    public MotorEx horizontalMotor = new MotorEx();
    double extendoPower = 0;
    motionProfile profile = new motionProfile(508, 100, 35, 440, 0.15);

    //servos
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoImplEx gripServo;
    public Servo linerRailServo;
    public ServoDegrees nest = new ServoDegrees();
    public DcMotor railEncoder;

    /**
     * linear rail constants
     * */
    double currentRailPosition;
    double railTargetPosition;
    public double currentEncoderPosition;
    double lastAxonWirePos;
    final double spoolSize = 3.6;
    double railTimeToPosition;
    double rotationsForFullTravel = 20/(spoolSize*Math.PI);
    double ticksToCM =(Math.PI * spoolSize)/8192;
    double timePerCM = (double) 1100 / 20;
    ElapsedTime railTime = new ElapsedTime();
    boolean runningToPosition = false;

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 750 / 360;
    double microRoboticTime = (double) 900 / 360;
    double gripperOpenTime = 400;


    public AxonEncoder linearPosition = new AxonEncoder();

    /**states*/
    public enum fourBar{
        preCollect,
        collect,
        dropPullOut,
        dropNest,
        transferringStates,
        transferUp,
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
        grab
    }

    public enum Nest{
        sample,
        specimen
    }

    /**
     * collect position values
     * */
    double mainPivotCollect = 97;
    double secondPivotCollect = 15;

    /**
     * preCollect position values
     * */
    double mainPivotPreCollect = 116;
    double secondPivotPreCollect = 40;
    double rotatePreCollect = 135;

    /**
     * transfer up position values
     * */
    double mainPivotTransferUp = 155;
    double secondPivotTransferUp = 165;
    double rotateStow = 45;

    /**
     * stowed position values
     * */
    double mainPivotStow = 175;
    double secondPivotStow = 165;

    /**
     * stow position values
     * */
    double mainPivotTransInt = 90;
    double secondPivotTransInt = 190;
    double rotateTransInt = 135;

    /**
     * drop nest position values
     * */
    double mainPivotPlaceNest = 193;
    double secondPlaceNest = 203;
    double rotatePlaceNest = 47;

    /**
     * drop nest above position values
     * */
    double mainPivotDropNestPull = 153;
    double secondDropNestPull = 185;
    double rotateDropNestPull = 47;

    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;

    /**enum states*/
    private Nest nestState = Nest.sample;
    private fourBar fourBarState = fourBar.transferUp;
    private fourBar fourBarTargetState = fourBar.transferUp;
    private collection collectionState = collection.stowed;
    private slideState slidesState = slideState.manuel;
    private clawState clawsState = clawState.drop;



    PIDController adjustment = new PIDController(0.2, 0, 0.0005);

    public void setSlideTarget(double slideTarget) {
        this.slideTarget = slideTarget;
    }

    public double getSlideTarget() {
        return slideTarget;
    }

    double slideTarget;

    public Collection2(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }

    static FileWriter fWriter;

    static {
        try {
            fWriter = new FileWriter("/sdcard/railLogs.txt");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    @Override
    public void init() {
        horizontalMotor.initMotor("horizontalMotor", getOpModeEX().hardwareMap);

        fourBarMainPivot.initServo("fourBarMainPivot", getOpModeEX().hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot", getOpModeEX().hardwareMap);
        griperRotate.initServo("gripperRotate", getOpModeEX().hardwareMap);
        gripServo = getOpModeEX().hardwareMap.get(ServoImplEx.class, "gripServo");
        linerRailServo  = getOpModeEX().hardwareMap.get(ServoImplEx.class, "linearRailServo");
        nest.initServo("nest", getOpModeEX().hardwareMap);
        linearPosition.init(getOpModeEX().hardwareMap, "axon2");

        railEncoder = getOpModeEX().hardwareMap.get(DcMotor.class, "LB");

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(new PwmControl.PwmRange(500, 2500), 270);
        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 270);
        gripServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        nest.setRange(new PwmControl.PwmRange(500, 2500), 270);

        profile.isVertical(false);

        fourBarMainPivot.setDirection(Servo.Direction.REVERSE);
        fourBarMainPivot.setOffset(36);

        nest.setDirection(Servo.Direction.REVERSE);
        nest.setOffset(5);

        griperRotate.setOffset(-10);
        griperRotate.setPosition(135);

        Stowed.execute();

    }

    @Override
    public void execute() {
        executeEX();

        if (slidesState == slideState.profile){
            extendoPower = profile.followProfile(horizontalMotor.getCurrentPosition(), horizontalMotor.getCurrentVelocity());
            if (!profile.isSlideRunning()){
                slidesState = slideState.manuel;
            }
        } else if (slidesState == slideState.manuel) {

            if (Math.abs(slideTarget - horizontalMotor.getCurrentPosition()/(440/35)) > 0.5 && getSlideTarget() != 0){
                extendoPower = adjustment.calculate(slideTarget, horizontalMotor.getCurrentPosition()/(440/35));
                System.out.println("extendoPower PID" + extendoPower);
            }else if (horizontalMotor.getCurrentPosition() < 20){
                extendoPower = 0;
            } else {
//                extendoPower = -0.1;
            }
        }

        if (nestState == Nest.sample){
            nest.setPosition(135);
        } else if (nestState == Nest.specimen) {
            nest.setPosition(45);
        }

        if (clawsState == clawState.grab){
            gripServo.setPosition(0.57);
        } else if (clawsState == clawState.drop) {
            gripServo.setPosition(0);
        }

        double slidesthing = ((double) 35 /440);
        horizontalMotor.update(extendoPower);
//        System.out.println("extendoPower: " + extendoPower);
//        System.out.println("hor velocity " + horizontalMotor.getCurrentVelocity()*slidesthing);
        horizontalMotor.updateVelocity();
//        System.out.println("!profile.isSlideRunning() " + !profile.isSlideRunning());

        updateRailPosition();

//        if (fourBarState == fourBar.stowed){
//            griperRotate.setPosition(rotateStow);
//        }

    }

    private final Command preCollect = new Execute(
            () -> {
                System.out.println("Running preCollect command");

                fourBarMainPivot.setPosition(mainPivotPreCollect);
                fourBarSecondPivot.setPosition(secondPivotPreCollect);
//                griperRotate.setPosition(rotatePreCollect);
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

    private final Command pullOut = new Execute(
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
            }
    );

    public final Command camera = new Execute(
            () -> {
                fourBarMainPivot.setPosition(170);
                fourBarSecondPivot.setPosition(109);
                griperRotate.setPosition(rotateTransInt);
            }
    );

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {
//                if (horizontalMotor.getCurrentPosition() < 40){
//                    extendoPower = 0;
//                }else if (horizontalMotor.getCurrentPosition() > 40){
//                    extendoPower = 0.01;
//                }
            },
            () -> true
    );

    public Command collect(double slideTarget){
        profile.generateMotionProfile(slideTarget, horizontalMotor.getCurrentPosition());
        slidesState = slideState.profile;
        this.slideTarget = slideTarget;
        return collect;
    }

    public Command cameraSetPoint(double slideTarget){
        profile.generateMotionProfile(slideTarget, horizontalMotor.getCurrentPosition());
        slidesState = slideState.profile;
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

    public Command transfer = new LambdaCommand(
            () -> {
                if(horizontalMotor.getCurrentPosition() > 30){
                    profile.generateMotionProfile(0, horizontalMotor.getCurrentPosition());
                    setSlideTarget(0);
                }
            },
            () -> {

                if (fourBarState == fourBar.collect && clawsState == clawState.drop){

                    clawsState = clawState.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                } else if (fourBarState == fourBar.collect && clawsState == clawState.grab){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotTransInt)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    transInt.execute();

                } else if (fourBarState == fourBar.transferInt) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()- mainPivotTransferUp)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()- secondPivotTransferUp)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    slidesState = slideState.profile;
                    transferUp.execute();

                } else if (fourBarState == fourBar.transferUp && griperRotate.getPositionDegrees() > 120) {

                    fourBarTimer.reset();
                    transferWaitTime = 500;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    griperRotate.setPosition(rotatePlaceNest);

                } else if (fourBarState == fourBar.transferUp && griperRotate.getPositionDegrees() < 55) {

                    fourBarTimer.reset();
//                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotPlaceNest)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPlaceNest)*microRoboticTime);
                    transferWaitTime = 200;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.dropNest;

                    dropNest.execute();

                }else if (fourBarState == fourBar.dropNest && clawsState == clawState.grab) {

                    clawsState = clawState.drop;

                    fourBarTimer.reset();
                    transferWaitTime = 200;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.dropNest;

                    dropNest.execute();

                }else if (fourBarState == fourBar.dropNest && clawsState == clawState.drop) {

                    fourBarTimer.reset();
                    transferWaitTime = 350;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.dropPullOut;

                    pullOut.execute();

                }else if (fourBarState == fourBar.dropPullOut && clawsState == clawState.drop) {

                    fourBarTimer.reset();
                    transferWaitTime = 90;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferUp;

                    transferUp.execute();

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.transferUp && clawsState == clawState.drop
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
        return currentRailPosition;
    }

    public void setRailTargetPosition(double targetPosition) {
        this.railTargetPosition = targetPosition;
        runToPosition();
        updateRailPosition();
    }

    public void updateRailPosition(){

        currentEncoderPosition = railEncoder.getCurrentPosition();
        currentRailPosition = currentEncoderPosition*ticksToCM;

        if (railTime.milliseconds() > railTimeToPosition && runningToPosition){
            linerRailServo.setPosition(0.5);
            runningToPosition = false;
        }else if (railTime.milliseconds() > railTimeToPosition && !runningToPosition){
            double delta = railTargetPosition - currentRailPosition;
            if (Math.abs(delta) > 0.5){
                runToPosition();
            }
        }

    }

    private void runToPosition(){

        double delta = railTargetPosition - currentRailPosition;

        railTimeToPosition = Math.abs(delta)*timePerCM;
        railTime.reset();

        if (delta > 0){
            linerRailServo.setPosition(1);
        }else {
            linerRailServo.setPosition(0);
        }

        runningToPosition = true;

        updateRailPosition();
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

    public Collection2.slideState getSlideState() {
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
        linerRailServo.getController().pwmDisable();
        nest.disableServo();
    }

}
