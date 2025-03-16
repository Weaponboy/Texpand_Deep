package dev.weaponboy.command_library.Subsystems;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.Execute;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
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

    public MotorEx slideMotor = new MotorEx();
    public MotorEx slideMotor2 = new MotorEx();

    boolean resettingSlides = false;
    boolean hold = false;

    public ServoDegrees griperSev =new ServoDegrees();
    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();
    public ServoDegrees griperRotateSev =new ServoDegrees();

    boolean drop = false;

    ElapsedTime dropTimer = new ElapsedTime();

    public TouchSensor slidesReset;

    public motionProfile profile = new motionProfile(1000, 210, 71, 900, 0.2);

    public TouchSensor clawSensor;

    public final double highBasket = 62;
    public final double autoHighBasket = 63.5;
    public final double lowBasket = 20;

    public final double highChamberFront = 25.5;
    public final double highChamberBack = 9;

    public final double visionTarget = 19.5;

    public final double chamberCollectScanPosition = 25.5;

    PIDController adjustment = new PIDController(0.012, 0, 0.01);

    public void setSlideDisabledForHang(boolean slideDisabledForHang) {
        this.slideDisabledForHang = slideDisabledForHang;
    }

    boolean slideDisabledForHang = false;

    double gripperDrop = 108;
    double gripperGrab = 52;
    double gripperSlightRelease = 80;

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
    double mainPivotSampleTransfer = 248;
    double secondSampleTransfer = 107;

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
    double gripperDepo = gripperGrab;

    /**
     * Bucket deposit position values
     * */
    double mainPivotDepoAuto = 92;
    double secondDepoAuto = 224;

    /**
     * Bucket deposit position values
     * */
    double mainPivotDepoAutoPreload  = 92;
    double secondDepoAutoPreload = 255;

    /**
     * Clipping position values
     * */
    double mainPivotClipFront = 215;
    double secondClipFront = 130;
    double gripperClipFront = gripperSlightRelease;

    /**
     * Clipping position values
     * */
    double mainPivotClipBack = 145;
    double secondClipBack = 230;
    double gripperClipBack = gripperGrab;

    /**
     * PRE clipping position values
     * */
    double mainPivotPreClipFront = 190;
    double secondPreClipFront = 130;
    double gripperPreClipFront = gripperGrab;

    /**
     * PRE clipping position values for clipping out the back
     * */
    double mainPivotPreClipBack = 100;
    double secondPreClipBack = 255;
    double gripperPreClipBack = gripperGrab;

    /**
     * PRE clipping position values for clipping out the back
     * */
    double mainPivotIntClipBack = 180;
    double secondPreIntClipBack = 285;

    /**
     * Hang
     * */
    double mainPivotHang = 100;
    double secondHang = 210;

    public boolean transferFailed = false;

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
        slightRelease
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

    boolean retracting = false;
    int counter = 0;

    BooleanSupplier holdCondition;
    BooleanSupplier holdingUntil;

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
                mainPivot.setPosition(mainPivotTransfer);
                secondPivot.setPosition(secondTransfer);
            }
    );

    public Command Hang = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotHang);
                secondPivot.setPosition(secondHang);
            }
    );

    private Command TransferSample = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotSampleTransfer);
                secondPivot.setPosition(secondSampleTransfer);
            }
    );

    public Command Deposit = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotDepo);
                secondPivot.setPosition(secondDepo);
                griperSev.setPosition(gripperDepo);
            }
    );

    public Command DepositAuto = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotDepoAuto);
                secondPivot.setPosition(secondDepoAuto);
            }
    );

    public Command transferHold(BooleanSupplier holdCondition){
        this.holdCondition = holdCondition;
        holdingUntil = () -> getCurrentCommand() != transfer;
        return transfer;
    }

    public LambdaCommand transfer = new LambdaCommand(
            () -> {
                runResetHold();
                Transfer.execute();
                fourbarState = fourBarState.transfer;
            },
            () -> {

            },
            () -> !resettingSlides
    );

    public LambdaCommand transferSample = new LambdaCommand(
            () -> {
                runResetHold();
                TransferSample.execute();
                fourbarState = fourBarState.transfer;
            },
            () -> {

            },
            () -> !resettingSlides
    );

    public Command PreClipFront = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotPreClipFront);
                secondPivot.setPosition(secondPreClipFront);
                griperSev.setPosition(gripperPreClipFront);
            }
    );

    private Command ClipFront = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotClipFront);
                secondPivot.setPosition(secondClipFront);
//                griperSev.setPosition(gripperClipFront);
            }
    );

    public Command PreClipBack = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotPreClipBack);
                secondPivot.setPosition(secondPreClipBack);
                griperSev.setPosition(gripperPreClipBack);
            }
    );

    public Command IntClipBack = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotIntClipBack);
                secondPivot.setPosition(secondPreIntClipBack);
            }
    );

    private Command ClipBack = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotClipBack);
                secondPivot.setPosition(secondClipBack);
                griperSev.setPosition(gripperClipBack);
            }
    );

    public Command deposit = new LambdaCommand(
            () -> {},
            () -> {

                if (fourbarState == fourBarState.basketDeposit && gripperState == gripper.drop){

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*5, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*5);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    slideSetPoint(0);
                    slides = Delivery.slideState.moving;

                    griperRotateSev.setPosition(90);
                    Transfer.execute();

                } else if (fourbarState == fourBarState.transfer && slideMotor.getCurrentPosition() > 150) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotDepo)*2, Math.abs(secondPivot.getPositionDegrees()-secondDepo)*2);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;

                    Deposit.execute();

                }else if (fourbarState == fourBarState.basketDeposit && gripperState == gripper.grab) {

                    gripperState = gripper.drop;

                    fourBarTimer.reset();
                    transferWaitTime = 100;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;

                    Deposit.execute();
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
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*5, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*5);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    slideSetPoint(0);
                    slides = Delivery.slideState.moving;

                    Transfer.execute();

                } else if (fourbarState == fourBarState.transfer && slideMotor.getCurrentPosition() > 150) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotDepo)*2, Math.abs(secondPivot.getPositionDegrees()-secondDepo)*2);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;

                    DepositAuto.execute();

                }else if (fourbarState == fourBarState.basketDeposit && gripperState == gripper.grab) {
                    secondPivot.setPosition(255);
                    drop = true;
                    dropTimer.reset();

                    fourBarTimer.reset();
                    transferWaitTime = 50;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;


                }
                if (drop && dropTimer.milliseconds()>35){
                    gripperState = gripper.drop;
                    drop = false;

                }


                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> !(fourbarState == fourBarState.transferringStates) && fourBarTimer.milliseconds() > transferWaitTime
    );

   public Command closeGripper = new LambdaCommand(
           () -> {
                fourBarTimer.reset();
                transferWaitTime = 50;
           },
           () -> {
                gripperState = gripper.grab;
           },
           () -> fourBarTimer.milliseconds() > transferWaitTime
   );

    public Command closeGripperFailSafe = new LambdaCommand(
            () -> {
                fourBarTimer.reset();
                transferWaitTime = 120;
                transferFailed = false;
            },
            () -> {
                if (fourBarTimer.milliseconds() > transferWaitTime && !clawSensor.isPressed()){
                    transferFailed = true;
                    gripperState = gripper.drop;
                }else {
                    gripperState = gripper.grab;
                }
            },
            () -> fourBarTimer.milliseconds() > transferWaitTime && clawSensor.isPressed() || transferFailed
    );

    public Command closeGripperSample = new LambdaCommand(
            () -> {
                fourBarTimer.reset();
                transferWaitTime = 200;
            },
            () -> {
                gripperState = gripper.grab;
            },
            () -> fourBarTimer.milliseconds() > transferWaitTime
    );

    public Command closeGripperSpec = new LambdaCommand(
            () -> {
                fourBarTimer.reset();
                transferWaitTime = 100;
            },
            () -> {
                gripperState = gripper.grab;
            },
            () -> fourBarTimer.milliseconds() > transferWaitTime
    );

    public Command stow = new LambdaCommand(
            () -> {
                gripperState = gripper.drop;
            },
            () -> {

                if (fourbarState != fourBarState.transferringStates) {
                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    Transfer.execute();
                    gripperState = gripper.drop;
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.transfer
    );

    public Command preClipFront = new LambdaCommand(
            () -> {
                slideSetPoint(highChamberFront);
                slides = slideState.moving;
            },
            () -> {

                if (getSlidePositionCM() > 10 && fourbarState != fourBarState.transferringStates) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- mainPivotPreClipFront)*12, Math.max(Math.abs(secondPivot.getPositionDegrees()- secondPreClipFront)*12, Math.abs(getSlidePositionCM() - slideTarget)*40));
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClip;

                    PreClipFront.execute();
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.preClip
    );

    public Command preClipBack = new LambdaCommand(
            () -> {
            },
            () -> {

                if (fourbarState != fourBarState.transferringStates && fourbarState != fourBarState.preClipInt) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- mainPivotPreClipBack)*4, Math.max(Math.abs(secondPivot.getPositionDegrees()- secondPreClipBack)*6, Math.abs(getSlidePositionCM() - slideTarget)*20));
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClipInt;

                    slideSetPoint(highChamberBack/2);
                    slides = slideState.moving;

                    PreClipBack.execute();

                }if (fourbarState == fourBarState.preClipInt) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- mainPivotIntClipBack)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()- secondPreIntClipBack)*axonMaxTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClip;

                    PreClipBack.execute();

                    slideSetPoint(highChamberBack);
                    slides = slideState.moving;
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.preClip && Math.abs(getSlidePositionCM() - highChamberBack) < 2
    );

    public Command preClipBackAuto = new LambdaCommand(
            () -> {
                slideSetPoint(highChamberBack-1);
                slides = slideState.moving;
            },
            () -> {

                if (fourbarState != fourBarState.transferringStates) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- mainPivotPreClipBack)*2, Math.max(Math.abs(secondPivot.getPositionDegrees()- secondPreClipBack)*4, 0));
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClip;

                    PreClipBack.execute();

                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.preClip && Math.abs(getSlidePositionCM() - (highChamberBack-1)) < 2
    );

    public Command cameraScan = new LambdaCommand(
            () -> {
                slideSetPoint(chamberCollectScanPosition);
                slides = slideState.moving;
            },
            () -> {

                if (getSlidePositionCM() > 15 && fourbarState != fourBarState.transferringStates) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- findCameraScanPosition(true, chamberCollectScanPosition))*25, Math.abs(secondPivot.getPositionDegrees()- secondPreClipFront)*25);
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
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- mainPivotClipFront)*9, Math.abs(secondPivot.getPositionDegrees()- secondClipFront)*9);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.clip;

                    slideSetPoint(highChamberFront-6);
                    slides = Delivery.slideState.moving;

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

    public Command clipBack = new LambdaCommand(
            () -> {
//                slideSetPoint(highChamberBack+5);
//                slides = slideState.moving;
            },
            () -> {

                if (fourbarState == fourBarState.preClip) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()- mainPivotClipBack)*5, Math.abs(secondPivot.getPositionDegrees()- secondClipBack)*5);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.clip;

                    ClipBack.execute();

                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.clip
    );

    public Command releaseClip = new LambdaCommand(
            () -> {},
            () -> {

                if(fourbarState == fourBarState.clip && gripperState == gripper.grab){

                    fourBarTimer.reset();
                    ClippingWaitTime = 200;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.clip;

                    gripperState = gripper.drop;

                }else if(fourbarState == fourBarState.clip){

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClipInt;

                    Transfer.execute();

                }else if(fourbarState == fourBarState.preClipInt){

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.transfer;

                    slideSetPoint(0);
                    slides = Delivery.slideState.moving;
                    griperRotateSev.setPosition(90);

                    Transfer.execute();
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.transfer
    );

    public Command releaseClipScan = new LambdaCommand(
            () -> {},
            () -> {

                if(fourbarState == fourBarState.clip && gripperState == gripper.grab){

                    fourBarTimer.reset();
                    ClippingWaitTime = 200;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.clip;

                    gripperState = gripper.drop;

                }else if(fourbarState == fourBarState.clip){

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.basketDeposit;

                    cameraScan.execute();

                }else if(fourbarState == fourBarState.basketDeposit){

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClipInt;

                    slideSetPoint(chamberCollectScanPosition);
                    slides = Delivery.slideState.moving;
                    griperRotateSev.setPosition(90);
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.preClipInt
    );

    public void slideSetPoint(double targetPosition){
        slideTarget = targetPosition;
    }

    @Override
    public void init() {

        slideMotor.initMotor("slideMotor", getOpModeEX().hardwareMap);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2.initMotor("slideMotor2", getOpModeEX().hardwareMap);
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
        secondPivot.setRange(335);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        griperSev.setOffset(-10);
        griperSev.setPosition(gripperGrab);
        setGripperState(gripper.grab);

        mainPivot.setOffset(4.9);
        mainPivot.setPosition(mainPivotTransfer);
        secondPivot.setPosition(secondTransfer);

        griperRotateSev.setOffset(40);
        griperRotateSev.setPosition(90);

        Deposit.execute();

        profile.isVertical(true);

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

            if (error > 5 && !resettingSlides){
                slidePower = Range.clip(adjustment.calculate(((slideTarget+1) * ticksPerCM), currentPosition), -1, 1);
            }else if (slidesReset.isPressed()){
                slidePower = 0;
                if(slideTarget == 0){
                    resettingSlides = false;
                }
            }


        }
//
//        else if (slides == slideState.holdPosition && !resettingSlides){
//            if (Math.abs(slideMotor.getCurrentPosition())>70){
//                slidePower = 0.00005;
//            }else if(Math.abs(slideMotor.getCurrentPosition())>300){
//                slidePower = 0.00055;
//            }else if(Math.abs(slideMotor.getCurrentPosition())>500){
//                slidePower = 0.00075;
//            }else if(Math.abs(slideMotor.getCurrentPosition())>750){
//                slidePower = 0.0008;
//            }else {
//                slidePower = 0;
//            }
//        }

//        if (error < 5 && !resettingSlides){
//            slides = slideState.holdPosition;
//            slidePower = 0;
//        }

//        if (!resettingSlides && slideTarget == 0 && slideMotor.getVelocity() < 10 && !slidesReset.isPressed() && slideMotor.getCurrentPosition() > 50) {
//            resettingSlides = true;
//            slidePower = -0.7;
//        } else

        if (resettingSlides && slidesReset.isPressed()){

            slidePower = 0;

            resettingSlides = false;

            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

//        if (hold && holdingUntil.getAsBoolean()){
//            slidePower = 0;
//            hold = false;
//        }

        if (gripperState == Delivery.gripper.grab){
            griperSev.setPosition(gripperGrab);
        } else if (gripperState == Delivery.gripper.drop) {
            griperSev.setPosition(gripperDrop);
        } else if (gripperState == gripper.slightRelease) {
            griperSev.setPosition(gripperSlightRelease);
        }

        if (!slideDisabledForHang){
            slideMotor.update(Range.clip(slidePower, -0.7, 1));
            slideMotor2.update(Range.clip(slidePower, -0.7, 1));
        }

    }

    public double findCameraScanPosition(){

        double pivotHeight = getSlidePositionCM() + 42.5;

        double X = Math.sqrt((pivotHeight*pivotHeight)+(30 * 30));

        double firstAngle = Math.toDegrees(Math.acos(pivotHeight / X));
        double secondAngle = 180 - Math.toDegrees(Math.asin(8 * Math.sin(87.6) / X)) - 87.6;

        return 176 - (((firstAngle  + secondAngle)-90) * 0.794);

    }

//    public double findCameraScanPosition(boolean chamberCollect){
//
//        double pivotHeight = getSlidePositionCM() + 43;
//
//        double X = Math.sqrt((pivotHeight*pivotHeight)+(29 * 29));
//
//        double angleInRadians = Math.acos(8 * Math.sin(80) / X);
//
//        if (chamberCollect){
//            return 187.5 - ((Math.toDegrees(angleInRadians) + Math.toDegrees(Math.atan(29/pivotHeight))-90)*0.794);
//        }else {
//            return 191.5 - ((Math.toDegrees(angleInRadians) + Math.toDegrees(Math.atan(29/pivotHeight))-90)*0.794);
//        }
//
//    }

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
        slidePower = -0.7;
    }

    public void runResetHold(){
        slideTarget = 0;
        resettingSlides = true;
        slidePower = -0.7;
//        hold = true;
    }

    public double getSlidePositionCM(){
        return (((double) (slideMotor.getCurrentPosition() + slideMotor2.getCurrentPosition()) /2) * profile.CMPerTick)-1;
    }
}









