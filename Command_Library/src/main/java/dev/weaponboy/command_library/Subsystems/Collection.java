package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.command_library.Hardware.collectionTarget;
import dev.weaponboy.command_library.Hardware.motionProfile;

public class Collection extends SubSystem{

    //motion profile
    motionProfile profile = new motionProfile(1400, 140, 54, 1720, 0.15);

    /**
     * linear rail constants
     * */
    double currentRailPosition;
    double railTargetPosition;
    double currentAxonWirePos;
    double lastAxonWirePos;
    final double spoolSize = 3.6; //in cm
    double railTimeToPosition;
    double rotationsForFullTravel = 20/(spoolSize*Math.PI);
    double timeForFullRotation = 400; // in ms
    double timePerCM = (rotationsForFullTravel*timeForFullRotation)/20;
    ElapsedTime railTime = new ElapsedTime();
    boolean runningToPosition = false;

    double waitForTransfer;
    double timePerDegree = (double) 720 /360;
    ElapsedTime initialTransfer = new ElapsedTime();

    /**
     * transfer constants
     * */
    ElapsedTime transferTime = new ElapsedTime();
    boolean transferHappening = false;
    double gripperWaitTime = 500;

    /**
     * Motor and servos
     * */
    public MotorEx horizontalMotor = new MotorEx();
    double extendoPower = 0;

    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoImplEx gripServo;
    public Servo linerRailServo;
    public ServoDegrees nest = new ServoDegrees();

    public AxonEncoder linearPosition = new AxonEncoder();

    public enum fourBar{
        preCollect,
        collect,
        dropNest,
        transferring,
        stowed
    }

    public enum Nest{
        sample,
        specimen
    }

    public enum slideState{
        moving,
        manuel
    }

    public enum gripper{
        drop,
        grab
    }

    private slideState slidesState = slideState.manuel;
    private gripper gripperState = gripper.drop;
    private fourBar collectionState = fourBar.stowed;
    private Nest nestState = Nest.sample;

    double armLength = 9.6;
    double mainPivotHeight = 12.4;

    collectionTarget target = new collectionTarget();
    double lastClawY = 0;
    double currentClawY = 0;

    public final int maxSlideExtension = 640;

    public Collection(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, holdPosition);
    }

    @Override
    public void execute() {

        executeEX();

        horizontalMotor.update(extendoPower);

        updateRailPosition();

        if (nestState == Nest.sample){
            nest.setPosition(135);
        } else if (nestState == Nest.specimen) {
            nest.setPosition(45);
        }

        if (gripperState == gripper.grab){
            gripServo.setPosition(0.5);
            if (collectionState == fourBar.collect){
                transferTime.reset();
                transferHappening = true;
            }
        } else if (gripperState == gripper.drop) {
            gripServo.setPosition(0);
        }

        if (transferTime.milliseconds() > gripperWaitTime && transferHappening){
            queueCommand(transfer);
            transferHappening = false;
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

        stow.execute();

    }

    public Command kinModel(collectionTarget target){
        this.target = target;
        return kinModel;
    }

    public Command kinModel = new LambdaCommand(
            () -> {},
            () -> {
                double mainAngle = 0;
                double secondAngle = 0;
                double railTarget = 0;
                lastClawY = currentClawY;

                double secondPivotHeight = target.getZ() + armLength;

                if (secondPivotHeight > mainPivotHeight){
                    mainAngle = Math.toRadians(90) + (Math.toRadians(90) - (Math.acos((secondPivotHeight - mainPivotHeight) / armLength)));
                }else if (secondPivotHeight < mainPivotHeight){
                    mainAngle = Math.acos((secondPivotHeight - mainPivotHeight) / armLength);
                }

                currentClawY = armLength * Math.sin(mainAngle);
                secondAngle = 180 - Math.toDegrees(mainAngle);

                double targetPosition = currentClawY - lastClawY;
                //spool ==40mm
                // cir 12.56cm

                fourBarMainPivot.setPosition(mainAngle);
                fourBarSecondPivot.setPosition(secondAngle);


            },
            () -> true
    );

    public  Command holdPosition = new LambdaCommand(
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

    public  Command init = new LambdaCommand(
            () -> {
            },
            () -> {
                fourBarMainPivot.setPosition(160);
                fourBarSecondPivot.setPosition(128);
                griperRotate.setPosition(130);
            },
            () -> true
    );

   public Command collect = new LambdaCommand(
        () -> {},
        () -> {
            fourBarMainPivot.setPosition(96);
            fourBarSecondPivot.setPosition(6);
            collectionState = fourBar.collect;
        },
        () -> true
    );

    public Command dropNest = new LambdaCommand(
            () -> {},
            () -> {

                fourBarMainPivot.setPosition(191);
                fourBarSecondPivot.setPosition(189);
                griperRotate.setPosition(45);

                collectionState = fourBar.dropNest;
                setNestState(Nest.sample);

            },
            () -> true
    );

    public Command stow = new LambdaCommand(
            () -> {
                initialTransfer.reset();
                waitForTransfer = Math.abs(fourBarMainPivot.getPositionDegrees() - 175)*timePerDegree;
            },
            () -> {
                fourBarMainPivot.setPosition(175);
                fourBarSecondPivot.setPosition(180);

                if (initialTransfer.milliseconds() > waitForTransfer){
                    griperRotate.setPosition(45);
                    collectionState = fourBar.stowed;
                }
            },
            () -> initialTransfer.milliseconds() > waitForTransfer
    );

    public Command transfer = new LambdaCommand(
            () -> {
                initialTransfer.reset();
                waitForTransfer = Math.abs(fourBarSecondPivot.getPositionDegrees() - 190)*timePerDegree;
            },
            () -> {
                fourBarMainPivot.setPosition(90);
                fourBarSecondPivot.setPosition(190);
                griperRotate.setPosition(135);

                collectionState = fourBar.transferring;

                if (initialTransfer.milliseconds() > waitForTransfer){
                    queueCommand(stow);
                }
            },
            () -> initialTransfer.milliseconds() > waitForTransfer
    );

   public Command camera = new LambdaCommand(
            () -> {},
            () -> {
                fourBarMainPivot.setPosition(180);
                fourBarSecondPivot.setPosition(120);
                griperRotate.setPosition(90);
            },
            () -> true
    );

    public Command preCollect = new LambdaCommand(
            () -> {},
            () -> {
                fourBarMainPivot.setPosition(109);
                fourBarSecondPivot.setPosition(30);
                griperRotate.setPosition(90);
                collectionState = fourBar.preCollect;
            },
            () -> true
    );

    public LambdaCommand followMotionPro = new LambdaCommand(
            () -> {},
            ()-> {
                extendoPower = profile.followProfile(horizontalMotor.getCurrentPosition());
            },
            ()-> profile.isSlideRunning()
    );

    public void generateMotionProfile(double targetPosition){
        profile.generateMotionProfile(targetPosition, horizontalMotor.getCurrentPosition());
    }

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

        if ((lastAxonWirePos > 280 && currentAxonWirePos < 80) || (currentAxonWirePos > 280 && lastAxonWirePos < 80)){

            if (deltaPosition > 0){

                realDelta = findRealDelta(lastAxonWirePos, currentAxonWirePos);
                deltaCM = realDelta * cmPerDegree;
                currentRailPosition -= deltaCM;

            } else if (deltaPosition < 0) {
                realDelta = findRealDelta(lastAxonWirePos, currentAxonWirePos);
                deltaCM = realDelta * cmPerDegree;
                currentRailPosition += deltaCM;
            }

        } else {
            currentRailPosition += deltaPosition*cmPerDegree;
        }

        if (railTime.milliseconds() >= railTimeToPosition && runningToPosition){
            linerRailServo.setPosition(0.5);
            runningToPosition = false;
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

//        if (Math.abs(delta) < 0.2){
//            linerRailServo.setPosition(0.5);
//            runningToPosition = false;
//        }else if(Math.abs(delta) > 1 && !runningToPosition){
//
//
//        }else if (railTime.milliseconds() >= railTimeToPosition && runningToPosition){
//            linerRailServo.setPosition(0.5);
//            runningToPosition = false;
//        }

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

    public void disableServos(){
        fourBarMainPivot.disableServo();
        fourBarSecondPivot.disableServo();
        griperRotate.disableServo();
        gripServo.getController().pwmDisable();
        linerRailServo.getController().pwmDisable();
        nest.disableServo();
    }

    public void safePositions(){
        fourBarMainPivot.setPosition(180);
        fourBarSecondPivot.setPosition(160);
        griperRotate.setPosition(135);
        gripServo.setPosition(0.3);
        linerRailServo.setPosition(0.5);
        nest.setPosition(135);
    }

    public gripper getGripperState() {
        return gripperState;
    }

    public void setGripperState(gripper gripperState) {
        this.gripperState = gripperState;
    }

    public Nest getNestState() {
        return nestState;
    }

    public void setNestState(Nest nestState) {
        this.nestState = nestState;
    }

    public fourBar getCollectionState() {
        return collectionState;
    }

    public void setCollectionState(fourBar collectionState) {
        this.collectionState = collectionState;
    }

    public slideState getSlidesState() {
        return slidesState;
    }

    public void xsetSlidesState(slideState slidesState) {
        this.slidesState = slidesState;
    }

    public void setExtendoPower(double extendoPower) {
        this.extendoPower = extendoPower;
    }


}
