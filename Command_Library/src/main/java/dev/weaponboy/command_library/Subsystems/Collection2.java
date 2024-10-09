package dev.weaponboy.command_library.Subsystems;

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

public class Collection2 extends SubSystem {

    // slides
    public MotorEx horizontalMotor = new MotorEx();
    double extendoPower = 0;
    motionProfile profile = new motionProfile(1400, 140, 35, 440, 0.15);

    //servos
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoImplEx gripServo;
    public Servo linerRailServo;
    public ServoDegrees nest = new ServoDegrees();

    /**
     * linear rail constants
     * */
    double currentRailPosition;
    double railTargetPosition;
    public double currentAxonWirePos;
    double lastAxonWirePos;
    final double spoolSize = 3.6; //in cm
    double railTimeToPosition;
    double rotationsForFullTravel = 20/(spoolSize*Math.PI);
    double timeForFullRotation = 540; // in ms
    double timePerCM = (double) 1400 / 20;
    ElapsedTime railTime = new ElapsedTime();
    boolean runningToPosition = false;

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 690 / 360;
    double microRoboticTime = (double) 840 / 360;
    double gripperOpenTime = 400;


    public AxonEncoder linearPosition = new AxonEncoder();

    /**states*/
    public enum fourBar{
        preCollect,
        collect,
        dropPullOut,
        dropNest,
        transferringStates,
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

    /**
     * collect position values
     * */
    double mainPivotCollect = 96;
    double secondPivotCollect = 15;

    /**
     * preCollect position values
     * */
    double mainPivotPreCollect = 116;
    double secondPivotPreCollect = 40;
    double rotatePreCollect = 135;

    /**
     * stow position values
     * */
    double mainPivotStow = 155;
    double secondPivotStow = 165;
    double rotateStow = 45;

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
    private fourBar fourBarState = fourBar.stowed;
    private fourBar fourBarTargetState = fourBar.stowed;
    private collection collectionState = collection.stowed;
    private slideState slidesState = slideState.manuel;
    private clawState clawsState = clawState.drop;

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

        Stow.execute();

        currentAxonWirePos = linearPosition.getPosition();
        lastAxonWirePos = currentAxonWirePos;

    }

    @Override
    public void execute() {
        executeEX();

        if (slidesState == slideState.profile){
            extendoPower = profile.followProfile(horizontalMotor.getCurrentPosition());
            if (!profile.isSlideRunning()){
                slidesState = slideState.manuel;
            }
        } else if (slidesState == slideState.manuel) {
            if (horizontalMotor.getCurrentPosition() < 20){
                extendoPower = 0;
            }else {
                extendoPower = 0.1;
            }
        }

        if (clawsState == clawState.grab){
            gripServo.setPosition(0.57);
        } else if (clawsState == clawState.drop) {
            gripServo.setPosition(0);
        }

        horizontalMotor.updateVelocity();
        horizontalMotor.update(extendoPower);
        System.out.println("extendoPower: " + extendoPower);
        System.out.println("hor velocity " + horizontalMotor.getCurrentVelocity());
        System.out.println("!profile.isSlideRunning() " + !profile.isSlideRunning());

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
                griperRotate.setPosition(rotatePreCollect);
            }
    );

    private final Command Collect = new Execute(
            () -> {

                System.out.println("Running collect command");

                fourBarMainPivot.setPosition(mainPivotCollect);
                fourBarSecondPivot.setPosition(secondPivotCollect);
            }
    );

    private final Command Stow = new Execute(
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
                fourBarMainPivot.setPosition(180);
                fourBarSecondPivot.setPosition(120);
                griperRotate.setPosition(90);
            }
    );

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {
                if (horizontalMotor.getCurrentPosition() < 40){
                    extendoPower = 0;
                }else if (horizontalMotor.getCurrentPosition() > 40){
                    extendoPower = 0.01;
                }
            },
            () -> true
    );

    public Command collect(double slideTarget){
        profile.generateMotionProfile(slideTarget, horizontalMotor.getCurrentPosition());
        slidesState = slideState.profile;
        return collect;
    }

    public Command collect = new LambdaCommand(
            () -> {},
            () -> {

                System.out.println("Running collect command FULL");

                if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.collect;

                    Collect.execute();

                } else if (fourBarState == fourBar.collect) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.preCollect;

                    preCollect.execute();

                }else if (!(fourBarState == fourBar.transferringStates)){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
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
                    slidesState = slideState.profile;
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
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    Stow.execute();

                } else if (fourBarState == fourBar.stowed && griperRotate.getPositionDegrees() > 120) {

                    fourBarTimer.reset();
                    transferWaitTime = 500;
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    griperRotate.setPosition(rotatePlaceNest);

                } else if (fourBarState == fourBar.stowed && griperRotate.getPositionDegrees() < 55) {

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
                    fourBarTargetState = fourBar.stowed;

                    Stow.execute();

                }else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                }

            },
            () -> fourBarState == fourBar.stowed && clawsState == clawState.drop
    );

    public Command stow = new LambdaCommand(
            () -> {},
            () -> {

                if (fourBarState == fourBar.collect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    transInt.execute();

                }else if (fourBarState == fourBar.preCollect){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(griperRotate.getPositionDegrees()-rotateTransInt)*microRoboticTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.transferInt;

                    transInt.execute();

                }else if (fourBarState == fourBar.dropNest) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                }else if (fourBarState == fourBar.transferInt) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(fourBarMainPivot.getPositionDegrees()-mainPivotStow)*axonMaxTime, Math.abs(fourBarSecondPivot.getPositionDegrees()-secondPivotStow)*microRoboticTime);
                    fourBarState = fourBar.transferringStates;
                    fourBarTargetState = fourBar.stowed;

                    Stow.execute();

                } else if (fourBarState == fourBar.transferringStates) {

                    if (fourBarTimer.milliseconds() > transferWaitTime){
                        fourBarState = fourBarTargetState;
                    }

                    if (fourBarState == fourBar.stowed){
                        griperRotate.setPosition(rotatePlaceNest);
                    }

                }

            },
            () -> fourBarState == fourBar.stowed
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

        double lastPosition = currentRailPosition;
        lastAxonWirePos = currentAxonWirePos;

        currentAxonWirePos = linearPosition.getPosition();

        double deltaPosition = lastAxonWirePos - currentAxonWirePos;
        double realDelta;
        double deltaCM;

        double spoolSize = 10.676;
        double cmPerDegree = spoolSize / 360;

        try {
            fWriter.write(System.lineSeparator());
            fWriter.write( "Last rail: " + lastPosition);
            fWriter.write(System.lineSeparator());
            fWriter.write( "currentRailPosition: " + currentRailPosition);
            fWriter.write(System.lineSeparator());
            fWriter.write( "currentAxonWirePos: " + currentAxonWirePos);
            fWriter.write(System.lineSeparator());
            fWriter.write( "lastAxonWirePos: " + lastAxonWirePos);
            fWriter.write(System.lineSeparator());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        if ((lastAxonWirePos > 240 && currentAxonWirePos < 80) || (currentAxonWirePos > 240 && lastAxonWirePos < 80)){

            if (deltaPosition > 0){

                realDelta = findRealDelta(lastAxonWirePos, currentAxonWirePos);
                deltaCM = realDelta * cmPerDegree;
                currentRailPosition -= deltaCM;

                try {
                    fWriter.write( "realDelta > 0: " + realDelta);
                    fWriter.write(System.lineSeparator());
                    fWriter.write( "deltaCM > 0: " + deltaCM);
                    fWriter.write(System.lineSeparator());

//                    Last rail: 12.338867878787884
//                    currentRailPosition: 12.338867878787884
//                    currentAxonWirePos: 2.8363636363636364
//                    lastAxonWirePos: 354.1090909090909
//                    realDelta > 0: 8.727272727272712
//                    deltaCM > 0: 0.25881212121212077

                } catch (IOException e) {
                    throw new RuntimeException(e);
                }

            } else if (deltaPosition < 0) {
                realDelta = findRealDelta(lastAxonWirePos, currentAxonWirePos);
                deltaCM = realDelta * cmPerDegree;
                currentRailPosition += deltaCM;

                try {
                    fWriter.write( "realDelta < 0: " + realDelta);
                    fWriter.write(System.lineSeparator());
                    fWriter.write( "deltaCM < 0: " + deltaCM);
                    fWriter.write(System.lineSeparator());
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }

        } else {
            currentRailPosition += deltaPosition*cmPerDegree;

            try {
                fWriter.write( "deltaPosition normal: " + deltaPosition);
                fWriter.write(System.lineSeparator());
                fWriter.write( "deltaPosition*cmPerDegree: " + deltaPosition*cmPerDegree);
                fWriter.write(System.lineSeparator());

//                Last rail: 2.397247272727276
//                currentRailPosition: 2.397247272727276
//                currentAxonWirePos: 277.3090909090909
//                lastAxonWirePos: 5.563636363636364
//                deltaPosition normal: -271.74545454545455
//                deltaPosition*cmPerDegree: -8.058762424242424


            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

        if (railTime.milliseconds() > railTimeToPosition && runningToPosition){
            linerRailServo.setPosition(0.5);
            runningToPosition = false;
        }else if (railTime.milliseconds() > railTimeToPosition && !runningToPosition){
            double delta = railTargetPosition - currentRailPosition;
            if (Math.abs(delta) > 0.5){
                runToPosition();
            }
        }

        try {
            fWriter.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
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

    public void disableServos(){
        fourBarMainPivot.disableServo();
        fourBarSecondPivot.disableServo();
        griperRotate.disableServo();
        gripServo.getController().pwmDisable();
        linerRailServo.getController().pwmDisable();
        nest.disableServo();
    }

}
