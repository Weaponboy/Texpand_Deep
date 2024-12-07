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

    public ServoDegrees griperSev =new ServoDegrees();
    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();

    public TouchSensor slidesReset;

    public motionProfile profile = new motionProfile(1200, 210, 71, 1080, 0.2);

    public TouchSensor clawSensor;

    public final double highBasket = 68;
    public final double lowBasket = 20;

    public final double highChamber = 30;

    public final double chamberCollectScanPosition = 19.5;

    PIDController adjustment = new PIDController(0.015, 0, 0.00005);

    double gripperDrop = 120;
    double gripperGrab = 72;
    double gripperSlightRelease = 78;

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 690 / 360;
    double microRoboticTime = (double) 840 / 360;
    double gripperOpenTime = 400;

    /**
     * transfer position values
     * */
    double mainPivotTransfer = 260;
    double secondTransfer = 140;

    /**
     * bucket deposit position values
     * */
    double mainPivotDepo = 85;
    double secondDepo = 220;
    double gripperDepo = gripperGrab;

    /**
     * clipping position values
     * */
    double mainPivotClip = 205;
    double secondClip = 60;
    double gripperClip = gripperSlightRelease;

    /**
     * PRE clipping position values
     * */
    double mainPivotPreClip = 192;
    double secondPreClip = 70;
    double gripperPreClip = gripperGrab;

    public enum fourBarState {
        transfer,
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

    public Command Deposit = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotDepo);
                secondPivot.setPosition(secondDepo);
                griperSev.setPosition(gripperDepo);
            }
    );

    public LambdaCommand transfer = new LambdaCommand(
            () -> {
                runReset();
                Transfer.execute();
                fourbarState = fourBarState.transfer;
            },
            () -> {},
            () -> true
    );

    public Command PreClip = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotPreClip);
                secondPivot.setPosition(secondPreClip);
                griperSev.setPosition(gripperPreClip);
            }
    );

    private Command Clipping = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotClip);
                secondPivot.setPosition(secondClip);
                griperSev.setPosition(gripperClip);
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

                    Transfer.execute();

                } else if (fourbarState == fourBarState.transfer && slideMotor.getCurrentPosition() > 400) {

                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotDepo)*6, Math.abs(secondPivot.getPositionDegrees()-secondDepo)*6);
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

   public Command closeGripper = new LambdaCommand(
           () -> {
                fourBarTimer.reset();
                transferWaitTime = 200;
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

    public Command preClip = new LambdaCommand(
            () -> {
                slideSetPoint(highChamber);
                slides = slideState.moving;
            },
            () -> {

                if (getSlidePositionCM() > 10) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotPreClip)*12, Math.max(Math.abs(secondPivot.getPositionDegrees()-secondPreClip)*12, Math.abs(getSlidePositionCM() - slideTarget)*40));
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClip;
                    gripperState = gripper.slightRelease;

                    PreClip.execute();
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.preClip
    );

    public Command cameraScan = new LambdaCommand(
            () -> {
                slideSetPoint(chamberCollectScanPosition);
                slides = slideState.moving;
            },
            () -> {

                if (getSlidePositionCM() > 15) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotPreClip)*15, Math.abs(secondPivot.getPositionDegrees()-secondPreClip)*15);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClip;
                    gripperState = gripper.slightRelease;

                    PreClip.execute();
                    mainPivot.setPosition(findCameraScanPosition(true, chamberCollectScanPosition));
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.preClip
    );

    public Command Clip = new LambdaCommand(
            () -> {},
            () -> {

                if (fourbarState == fourBarState.preClip && slideMotor.getCurrentPosition() > 100) {

                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotClip)*8, Math.abs(secondPivot.getPositionDegrees()-secondClip)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.clip;
                    gripperState = gripper.slightRelease;

                    slideSetPoint(highChamber-9);
                    slides = Delivery.slideState.moving;

                    Clipping.execute();

                }else if(fourbarState == fourBarState.clip && gripperState == gripper.slightRelease){

                    fourBarTimer.reset();
                    ClippingWaitTime = 400;
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

        slideMotor.initMotor("slideMotor", getOpModeEX().hardwareMap);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2.initMotor("slideMotor2", getOpModeEX().hardwareMap);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mainPivot.initServo("mainPivot",getOpModeEX().hardwareMap);
        secondPivot.initServo("secondPivot",getOpModeEX().hardwareMap);
        griperSev.initServo("devClaw",getOpModeEX().hardwareMap);

        slidesReset = getOpModeEX().hardwareMap.get(TouchSensor.class, "DeliveryReset");
        clawSensor = getOpModeEX().hardwareMap.get(TouchSensor.class, "clawIR");

        griperSev.setRange(new PwmControl.PwmRange(500, 2500),180);
        mainPivot.setRange(335);
        secondPivot.setRange(335);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (clawSensor.isPressed()){
            griperSev.setPosition(gripperGrab);
            setGripperState(gripper.grab);
        }else {
            griperSev.setPosition(gripperDrop);
            setGripperState(gripper.drop);
        }

        griperSev.setPosition(gripperGrab);
        setGripperState(gripper.grab);

        mainPivot.setOffset(-2.1);
        mainPivot.setPosition(mainPivotTransfer);
        secondPivot.setPosition(secondTransfer);

        Deposit.execute();

        profile.isVertical(true);

        runReset();

    }

    @Override
    public void execute() {

        executeEX();

        double ticksPerCM = (double) 1080 / 71;
        double error;

        if ((slideTarget*ticksPerCM) < 1080){
            error = Math.abs((slideTarget*ticksPerCM) - (double) slideMotor.getCurrentPosition());
        }else {
            error = Math.abs((1080) - (double) slideMotor.getCurrentPosition());
        }

        if(slides == slideState.moving && !resettingSlides){

            if (error > 5 && !resettingSlides){
                slidePower = Range.clip(adjustment.calculate(((slideTarget+1) * ticksPerCM), slideMotor.getCurrentPosition()), -1, 1);
            }else if (slidesReset.isPressed()){
                slidePower = 0;
                if(slideTarget == 0){
                    resettingSlides = false;
                }
            }


        }else if (slides == slideState.holdPosition && !resettingSlides){
            if (Math.abs(slideMotor.getCurrentPosition())>70){
                slidePower = 0.00005;
            }else if(Math.abs(slideMotor.getCurrentPosition())>300){
                slidePower = 0.00055;
            }else if(Math.abs(slideMotor.getCurrentPosition())>500){
                slidePower = 0.00075;
            }else if(Math.abs(slideMotor.getCurrentPosition())>750){
                slidePower = 0.0008;
            }else {
                slidePower = 0;
            }
        }

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

        if (gripperState == Delivery.gripper.grab){
            griperSev.setPosition(gripperGrab);
        } else if (gripperState == Delivery.gripper.drop) {
            griperSev.setPosition(gripperDrop);
        } else if (gripperState == gripper.slightRelease) {
            griperSev.setPosition(gripperSlightRelease);
        }

        slideMotor.update(slidePower);
        slideMotor2.update(slidePower);
    }

    public double findCameraScanPosition(){

        double pivotHeight = getSlidePositionCM() + 43;

        double X = Math.sqrt((pivotHeight*pivotHeight)+(29 * 29));

        double angleInRadians = Math.acos(8 * Math.sin(80) / X);

        return 194 - ((Math.toDegrees(angleInRadians) + Math.toDegrees(Math.atan(29/pivotHeight))-90)*0.794);

    }

    public double findCameraScanPosition(boolean chamberCollect){

        double pivotHeight = getSlidePositionCM() + 43;

        double X = Math.sqrt((pivotHeight*pivotHeight)+(29 * 29));

        double angleInRadians = Math.acos(8 * Math.sin(80) / X);

        if (chamberCollect){
            return 193 - ((Math.toDegrees(angleInRadians) + Math.toDegrees(Math.atan(29/pivotHeight))-90)*0.794);
        }else {
            return 191.5 - ((Math.toDegrees(angleInRadians) + Math.toDegrees(Math.atan(29/pivotHeight))-90)*0.794);
        }

    }

    public double findCameraScanPosition(boolean chamberCollect, double fakeSlideHeight){

        double pivotHeight = fakeSlideHeight + 43;

        double X = Math.sqrt((pivotHeight*pivotHeight)+(29 * 29));

        double angleInRadians = Math.acos(8 * Math.sin(80) / X);

        if (chamberCollect){
            return 193 - ((Math.toDegrees(angleInRadians) + Math.toDegrees(Math.atan(29/pivotHeight))-90)*0.794);
        }else {
            return 191.5 - ((Math.toDegrees(angleInRadians) + Math.toDegrees(Math.atan(29/pivotHeight))-90)*0.794);
        }

    }

    public void runReset(){
        slideTarget = 0;
        resettingSlides = true;
        slidePower = -0.7;
    }

    public double getSlidePositionCM(){
        return (((double) (slideMotor.getCurrentPosition() + slideMotor2.getCurrentPosition()) /2) * profile.CMPerTick)-1;
    }
}









