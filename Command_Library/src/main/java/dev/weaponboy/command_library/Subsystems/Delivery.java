package dev.weaponboy.command_library.Subsystems;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.Execute;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.BooleanSupplier;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.command_library.Hardware.motionProfile;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

public class Delivery extends SubSystem {

    public double slideTarget = 0;
    double slidePower = 0;

    public DcMotorEx slideMotor;
    public DcMotorEx slideMotor2;

    boolean resettingSlides = false;

    public boolean isLowBucket() {
        return lowBucket;
    }

    public void setLowBucket(boolean lowBucket) {
        this.lowBucket = lowBucket;
    }

    boolean lowBucket = false;

    public ServoDegrees griperSev =new ServoDegrees();
    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();
    public ServoDegrees griperRotateSev =new ServoDegrees();

    boolean drop = false;

    public void setSpikeTransfer(boolean spikeTransfer) {
        this.spikeTransfer = spikeTransfer;
    }

    boolean spikeTransfer = false;

    ElapsedTime dropTimer = new ElapsedTime();

    public TouchSensor slidesReset;

    public motionProfile profile = new motionProfile(1000, 210, 71, 900, 0.2);

    public TouchSensor clawSensor;

    public double highBasket = 61;
    public final double autoHighBasket = 60.5;
    public final double lowBasket = 20;

    public final double spikeTransferHeight = 16.6;

    public final double chamberCollectScanPosition = 25.5;

    PIDController adjustment = new PIDController(0.012, 0, 0.01);

    public void setSlideDisabledForHang(boolean slideDisabledForHang) {
        this.slideDisabledForHang = slideDisabledForHang;
    }

    boolean slideDisabledForHang = false;

    public double gripperDrop = 68;
    double gripperGrab = 42;
    double gripperSlightRelease = 65;
    double gripperTightGrab = 38;

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 690 / 360;
    double microRoboticTime = (double) 840 / 360;
    double gripperOpenTime = 400;

    /**
     * transfer position values
     * */
    double mainPivotTransfer = 244;
    double secondTransfer = 162;

    /**
     * transfer position values
     * */
    double mainPivotSpikeTransfer = 235;
    double secondSpikeTransfer = 112;

    /**
     * transfer position values
     * */
    double mainPivotCamara = 210;
    double secondCamara = 107;

    /**
     * Bucket deposit position values
     * */
    double mainPivotDepo = 94;
    double secondDepo = 228;
    double gripperDepo = 40;

    /**
     * Bucket deposit position values
     * */
    double mainPivotDepoAuto = 92;
    double secondDepoAuto = 232;
    boolean runningStow = false;

    /**
     * Clipping position values
     * */
    double mainPivotClipFront = 250;
    double secondClipFront = 115;

    double mainPivotClipFrontInt = 200;
    double secondClipFrontInt = 115;

    /**
     * PRE clipping position values
     * */
    double mainPivotPreClipFront = 190;
    double secondPreClipFront = 80;
    double gripperPreClipFront = gripperGrab;

    /**
     * Hang
     * */
    double mainPivotHang = 100;
    double secondHang = 210;

    public enum fourBarState {
        transfer,
        preClipInt,
        preClip,
        clip,
        basketDeposit,
        transferringStates
    }

    public enum gripper{
        drop,
        grab,
        slightRelease,
        tightGrab
    }

    public gripper getGripperState() {
        return gripperState;
    }

    public void setGripperState(gripper gripperState) {
        this.gripperState = gripperState;
    }

    private gripper gripperState = gripper.grab;
    public Delivery.fourBarState fourbarState = fourBarState.transfer;
    public Delivery.fourBarState fourBarTargetState = fourBarState.transfer;
    public Delivery.slideState slides = slideState.holdPosition;

    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;
    double ClippingWaitTime;

    public enum slideState {
        holdPosition,
        moving,
    }

    public Delivery(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, holdPosition);
    }

    public LambdaCommand holdPosition = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> true
    );

    private Command Transfer = new Execute(
            () -> {
                if (spikeTransfer){
                    mainPivot.setPosition(mainPivotSpikeTransfer);
                    secondPivot.setPosition(secondSpikeTransfer);
                }else {
                    mainPivot.setPosition(mainPivotTransfer);
                    secondPivot.setPosition(secondTransfer);
                }
            }
    );

    public Command Hang = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotHang);
                secondPivot.setPosition(secondHang);
            }
    );

    public Command Deposit = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotDepo);
                secondPivot.setPosition(secondDepo);
            }
    );

    public Command DepositAuto = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotDepoAuto);
                secondPivot.setPosition(secondDepoAuto);
            }
    );

    private Command ClipFront = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotClipFront);
                secondPivot.setPosition(secondClipFront);
//                griperSev.setPosition(gripperClipFront);
            }
    );

    private Command ClipFrontInt = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotClipFrontInt);
                secondPivot.setPosition(secondClipFrontInt);
//                griperSev.setPosition(gripperClipFront);
            }
    );

    public Command deposit = new LambdaCommand(
            () -> {},
            () -> {

                if (fourbarState == fourBarState.basketDeposit && gripperState == gripper.drop){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*2.5, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*2.5);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    if (spikeTransfer){
                        slideSetPoint(spikeTransferHeight);
                        slides = Delivery.slideState.moving;
                    } else{
                        slideSetPoint(0);
                        runReset();
                    }

                    griperRotateSev.setPosition(90);
                    Transfer.execute();

                } else if (fourbarState == fourBarState.transfer && slideMotor.getCurrentPosition() > 150) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotDepo)*1.5, Math.abs(secondPivot.getPositionDegrees()-secondDepo)*1.5);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;

                    if (lowBucket){
                        mainPivot.setPosition(mainPivotDepo);
                        secondPivot.setPosition(secondDepo + 15);
                        gripperState = gripper.tightGrab;
                    }else {
                        Deposit.execute();
                        gripperState = gripper.tightGrab;
                    }

                }else if (fourbarState == fourBarState.basketDeposit && (gripperState == gripper.tightGrab)) {

                    gripperState = gripper.drop;

                    fourBarTimer.reset();
                    transferWaitTime = 100;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;

//                    if (lowBucket){
//                        mainPivot.setPosition(mainPivotDepo);
//                        secondPivot.setPosition(secondDepo + 15);
//                    }else {
//                        Deposit.execute();
//                    }
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> !(fourbarState == fourBarState.transferringStates) && fourBarTimer.milliseconds() > transferWaitTime
    );

    public Command depositAuto = new LambdaCommand(
            () -> {},
            () -> {

                if (fourbarState == fourBarState.basketDeposit && gripperState == gripper.drop){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*2.5, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*2.5);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    if (spikeTransfer){
                        slideSetPoint(spikeTransferHeight);
                    } else{
                        slideSetPoint(0);
                    }
                    slides = Delivery.slideState.moving;

                    Transfer.execute();

                } else if (fourbarState == fourBarState.transfer && slideMotor.getCurrentPosition() > 150) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotDepo)*2, Math.abs(secondPivot.getPositionDegrees()-secondDepo)*1.6);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;

                    mainPivot.setPosition(mainPivotDepo - 16);
                    secondPivot.setPosition(secondDepo + 8);
                    gripperState = gripper.tightGrab;

                }else if (fourbarState == fourBarState.basketDeposit && (gripperState == gripper.grab || gripperState == gripper.tightGrab)) {
                    secondPivot.setPosition(260);
                    drop = true;
                    dropTimer.reset();

                    fourBarTimer.reset();
                    transferWaitTime = 200;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;
                }

                if (drop && dropTimer.milliseconds() > 60){
                    gripperState = gripper.drop;
                    drop = false;
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> !(fourbarState == fourBarState.transferringStates) && fourBarTimer.milliseconds() > transferWaitTime
    );

    public Command openGripper = new LambdaCommand(
            () -> {
                fourBarTimer.reset();
                transferWaitTime = 120;
            },
            () -> {
                gripperState = gripper.drop;
            },
            () -> fourBarTimer.milliseconds() > transferWaitTime
    );

    public Command closeGripperSpike = new LambdaCommand(
            () -> {
                fourBarTimer.reset();
                mainPivot.setPosition(mainPivotSpikeTransfer - 15);
                secondPivot.setPosition(secondSpikeTransfer - 8);
//                slideSetPoint(getSlidePositionCM() - 0.8);
                transferWaitTime = 180;
            },
            () -> {
                if (fourBarTimer.milliseconds() > 100){
                    gripperState = gripper.grab;
                }
            },
            () -> fourBarTimer.milliseconds() > transferWaitTime
    );

    public Command closeGripperSpikeSpike = new LambdaCommand(
            () -> {
                fourBarTimer.reset();
                mainPivot.setPosition(mainPivotSpikeTransfer - 8);
                secondPivot.setPosition(secondSpikeTransfer - 7);
//                slideSetPoint(getSlidePositionCM() - 0.8);
                transferWaitTime = 180;
            },
            () -> {
                if (fourBarTimer.milliseconds() > 100){
                    gripperState = gripper.grab;
                }
            },
            () -> fourBarTimer.milliseconds() > transferWaitTime
    );

    public Command stow = new LambdaCommand(
            () -> {
                gripperState = gripper.drop;
                runningStow = false;
            },
            () -> {

                if (fourbarState != fourBarState.transferringStates) {
                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    runningStow = true;
                    Transfer.execute();
                    gripperState = gripper.drop;
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.transfer && runningStow
    );

    public Command spike = new LambdaCommand(
            () -> {
                gripperState = gripper.drop;
                slideSetPoint(spikeTransferHeight);
                slides = slideState.moving;
                fourbarState = fourBarState.preClipInt;
            },
            () -> {

                if (fourbarState != fourBarState.transferringStates) {
                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    Transfer.execute();
                    gripperState = gripper.drop;
                    slideSetPoint(spikeTransferHeight);
                    slides = slideState.moving;
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.transfer
    );

    public Command cameraScan = new LambdaCommand(
            () -> {
                slideSetPoint(chamberCollectScanPosition);
                slides = slideState.moving;
            },
            () -> {

                if (getSlidePositionCM() > 15 && fourbarState != fourBarState.transferringStates) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees() - findCameraScanPosition(true, chamberCollectScanPosition))*25, Math.abs(secondPivot.getPositionDegrees()- secondPreClipFront)*25);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClipInt;
                    gripperState = gripper.slightRelease;

                    secondPivot.setPosition(secondCamara);
                    mainPivot.setPosition(mainPivotCamara);
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.preClipInt && gripperState == gripper.slightRelease
    );

    public Command clipFront = new LambdaCommand(
            () -> {},
            () -> {

                if (fourbarState == fourBarState.preClip && slideMotor.getCurrentPosition() > 100) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- mainPivotClipFrontInt)*9, Math.abs(secondPivot.getPositionDegrees()- secondClipFrontInt)*9);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClipInt;

                    ClipFrontInt.execute();

                }else if(fourbarState == fourBarState.preClipInt){

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- mainPivotClipFront)*9, Math.abs(secondPivot.getPositionDegrees()- secondClipFront)*9);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.clip;

                    ClipFront.execute();

                }else if(fourbarState == fourBarState.clip && gripperState == gripper.grab){

                    fourBarTimer.reset();
                    ClippingWaitTime = 100;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.clip;

                    gripperState = gripper.drop;

                }else if(fourbarState == fourBarState.clip && slideMotor.getCurrentPosition() > 100 && gripperState == gripper.drop){

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    slideSetPoint(0);
                    slides = Delivery.slideState.moving;

                    Transfer.execute();
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.transfer
    );

    public void slideSetPoint(double targetPosition){
        slideTarget = targetPosition;
    }

    @Override
    public void init() {

        slideMotor = getOpModeEX().hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2 = getOpModeEX().hardwareMap.get(DcMotorEx.class, "slideMotor2");
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mainPivot.initServo("mainPivot",getOpModeEX().hardwareMap);
        secondPivot.initServo("secondPivot",getOpModeEX().hardwareMap);
        griperSev.initServo("devClaw",getOpModeEX().hardwareMap);
        griperRotateSev.initServo("devClawRotate",getOpModeEX().hardwareMap);

        slidesReset = getOpModeEX().hardwareMap.get(TouchSensor.class, "DeliveryReset");
        clawSensor = getOpModeEX().hardwareMap.get(TouchSensor.class, "clawIR");

        griperSev.setRange(new PwmControl.PwmRange(500, 2500),180);
        griperRotateSev.setRange(new PwmControl.PwmRange(500, 2500),180);
        mainPivot.setRange(335);
        secondPivot.setOffset(3);
        secondPivot.setRange(335);

        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        griperSev.setOffset(-8);
        griperSev.setPosition(gripperGrab);
        setGripperState(gripper.grab);

        mainPivot.setOffset(4.9);
        mainPivot.setPosition(mainPivotTransfer);
        secondPivot.setPosition(secondTransfer);

        griperRotateSev.setOffset(1);
        griperRotateSev.setPosition(90);

//        Deposit.execute();

        profile.isVertical(true);

//        mainPivot.setPosition(mainPivotSpikeTransfer);
//        secondPivot.setPosition(secondSpikeTransfer);

        runReset();

    }

    @Override
    public void execute() {

        executeEX();

        double ticksPerCM = (double) 900 / 71;
        double error;

        if ((slideTarget*ticksPerCM) < 900){
            error = Math.abs((slideTarget*ticksPerCM) - (double) slideMotor.getCurrentPosition());
        }else {
            error = Math.abs((90) - (double) slideMotor.getCurrentPosition());
        }

        double currentPosition = 0;

        if(slides == slideState.moving && !resettingSlides){

            if (slideMotor.getCurrentPosition() == 0){
                currentPosition = slideMotor2.getCurrentPosition();
            }else {
                currentPosition = slideMotor.getCurrentPosition();
            }

            if (error > 2 && !resettingSlides){
                slidePower = Range.clip(adjustment.calculate(((slideTarget+1) * ticksPerCM), currentPosition), -1, 1);
            }else if (slidesReset.isPressed()){
                slidePower = 0;
                if(slideTarget == 0){
                    resettingSlides = false;
                }
            }

        }

        if (resettingSlides && slidesReset.isPressed()){

            slidePower = 0;

            resettingSlides = false;

            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gripperState == Delivery.gripper.grab){
            griperSev.setPosition(gripperGrab);
        } else if (gripperState == Delivery.gripper.drop) {
            griperSev.setPosition(gripperDrop);
        } else if (gripperState == gripper.slightRelease) {
            griperSev.setPosition(gripperSlightRelease);
        }else if (gripperState == gripper.tightGrab) {
            griperSev.setPosition(gripperTightGrab);
        }

        if (!slideDisabledForHang){
            slideMotor.setPower(Range.clip(slidePower, -0.8, 1));
            slideMotor2.setPower(Range.clip(slidePower, -0.8, 1));
        }

    }

    public double findCameraScanPosition(){

        double pivotHeight = getSlidePositionCM() + 42.5;

        double X = Math.sqrt((pivotHeight*pivotHeight)+(30 * 30));

        double firstAngle = Math.toDegrees(Math.acos(pivotHeight / X));
        double secondAngle = 180 - Math.toDegrees(Math.asin(8 * Math.sin(87.6) / X)) - 87.6;

        return 176 - (((firstAngle  + secondAngle)-90) * 0.794);

    }

    public double findCameraScanPosition(boolean chamberCollect, double fakeSlideHeight){

        double pivotHeight = fakeSlideHeight + 42.5;

        double X = Math.sqrt((pivotHeight*pivotHeight)+(30 * 30));

        double firstAngle = Math.toDegrees(Math.acos(pivotHeight / X));
        double secondAngle = 180 - Math.toDegrees(Math.asin(8 * Math.sin(87.6) / X)) - 87.6;

        return 176 - (((firstAngle + secondAngle)-90) * 0.794);

    }

    public void runReset(){
        slideTarget = 0;
        resettingSlides = true;
        slidePower = -1;
    }

    public void runResetHold(){
        slideTarget = 0;
        resettingSlides = true;
        slidePower = -1;
//        hold = true;
    }

    public double getSlidePositionCM(){
        return (((double) (slideMotor.getCurrentPosition() + slideMotor2.getCurrentPosition()) /2) * profile.CMPerTick);
    }

    public void disableServos(){
        griperSev.disableServo();
        griperRotateSev.disableServo();
        mainPivot.disableServo();
        secondPivot.disableServo();
    }
}









