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

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.command_library.Hardware.motionProfile;

public class Delivery extends SubSystem {

   public DcMotor slideMotor;
   public DcMotor slideMotor2;


    public ServoDegrees griperSev =new ServoDegrees();
    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();
    public ServoDegrees linierRail= new ServoDegrees();

    public TouchSensor slidesReset;
//1150
//312
    motionProfile profile = new motionProfile(1000, 210, 65, 611, 0.15);

    double topRailFullExtension = 0;
    double topRailAllTheWayIn = 335;


    public TouchSensor clawSensor;


    public final double highBasket = 66;

    public final double highChamber = 30;
    public final double lowChamber = 0;

    ElapsedTime transferTimer = new ElapsedTime();
    boolean flipBackTime;
    boolean gripTimer;
    double griperDrop = 90;
    double gripergrab = 180;

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 690 / 360;
    double microRoboticTime = (double) 840 / 360;
    double gripperOpenTime = 400;

    /**
     * behind transfer position values
     * */
    double mainPivotBehindTransfer = 278;
    double secondBehindTransfer = 70;
    double gripperBehindTransfer = griperDrop;

    /**
     * transfer position values
     * */
    double mainPivotTransfer = 260;
    double secondTransfer = 70;
    double gripperTransfer = gripergrab;

    /**
     * bucket deposit position values
     * */
    double mainPivotDepo = 100;
    double secondDepo = 240;
    double gripperDepo = gripergrab;

    /**
     * bucket deposit position values
     * */
    double mainPivotScan = 160;
    double secondScan = 170;

    /**
     * clipping position values
     * */
    double mainPivotClip =160;
    double secondClip =160;
    double gripperClip = gripergrab;

    /**
     * PRE clipping position values
     * */
    double mainPivotPreClip = 160;
    double secondPreClip = 160;
    double gripperPreClip = gripergrab;

    public enum DeliveryState{
        transfer,
        depositBasket,
        stowed,
        basket,
    }

    public enum fourBarState {
        behindNest,
        grabNest,
        postTransfer,
        basketDeposit,
        transferringStates,
        preClip,
        clip
    }

    public enum gripper{
        drop,
        grab
    }



    public gripper getGripperState() {
        return gripperState;
    }

    public void setGripperState(gripper gripperState) {
        this.gripperState = gripperState;
    }

    private gripper gripperState = gripper.drop;
    public Delivery.fourBarState fourbarState = fourBarState.behindNest;
    public Delivery.fourBarState fourBarTargetState = fourBarState.behindNest;
    public Delivery.DeliveryState depositState = DeliveryState.stowed;
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

    public Delivery.slideState slidesState = slideState.holdPosition;

    public Delivery(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, holdPosition);
    }

    public LambdaCommand followMotionPro = new LambdaCommand(
            () ->{},
            ()-> {
                slideMotor.setPower(profile.followProfile(slideMotor.getCurrentPosition()));
                slideMotor2.setPower(profile.followProfile(slideMotor2.getCurrentPosition()));
                System.out.println("slide motor power" + profile.followProfile(slideMotor.getCurrentPosition()));
            },
            ()-> !profile.isSlideRunning()
    );

    public LambdaCommand holdPosition = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> true
    );

    public Command behindNest = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotBehindTransfer);
                secondPivot.setPosition(secondBehindTransfer);
                griperSev.setPosition(gripperBehindTransfer);
                gripperState = gripper.drop;
            }
    );

    private Command Transfer = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotTransfer);
                secondPivot.setPosition(secondTransfer);
                griperSev.setPosition(gripperTransfer);
            }
    );

    public Command Deposit = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotDepo);
                secondPivot.setPosition(secondDepo);
                griperSev.setPosition(gripperDepo);
            }
    );

    public Command Scanning = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotScan);
                secondPivot.setPosition(secondScan);
            }
    );

   public LambdaCommand transfer = new LambdaCommand(
            () -> {},
            () -> {
                slideSetPonts(4.9);
                slides = slideState.moving;



                if (fourbarState == fourBarState.behindNest){

                    fourBarTimer.reset();
//                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    transferWaitTime = 1000;

                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.grabNest;
                    Transfer.execute();

                }else if (fourbarState == fourBarState.grabNest && gripperState == gripper.drop){

                    gripperState = gripper.grab;

                    fourBarTimer.reset();
                    transferWaitTime = gripperOpenTime;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.grabNest;

                    Transfer.execute();

                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourbarState = fourBarTargetState;
                }

            },
            () -> fourbarState == fourBarState.grabNest && gripperState == gripper.grab
    );

   public Command deposit = new LambdaCommand(
           () -> {
               if (slideMotor.getCurrentPosition() > 100){
                   genProfile(0);
               }
           },
           () -> {

               if (fourbarState == fourBarState.basketDeposit && gripperState == gripper.drop){
                   fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
//                   transferWaitTime = 500;
                   fourbarState = fourBarState.transferringStates;
                   fourBarTargetState = fourBarState.behindNest;

                   queueCommand(followMotionPro);
                   slidesState = Delivery.slideState.moving;

                   behindNest.execute();
               } else if (fourbarState == fourBarState.grabNest && slideMotor.getCurrentPosition() > 400) {
                   fourBarTimer.reset();
                   transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotDepo)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondDepo)*microRoboticTime);
//                   transferWaitTime = 500;
                   fourbarState = fourBarState.transferringStates;
                   fourBarTargetState = fourBarState.basketDeposit;

                   Deposit.execute();
               }else if (fourbarState == fourBarState.basketDeposit && gripperState == gripper.grab) {

                   gripperState = gripper.drop;

                   fourBarTimer.reset();
//                   transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                   transferWaitTime = 500;
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

    public Command stow = new LambdaCommand(
            () -> {},
            () -> {

                if (fourbarState == fourBarState.grabNest) {
                    fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
//                   transferWaitTime = 500;
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.behindNest;

                    behindNest.execute();
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > transferWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState == fourBarState.behindNest
    );

//   public LambdaCommand dropOff = new LambdaCommand(
//            () -> System.out.println("init"),
//            () -> {
//                mainPivot.setPosition(300);
//                secondPivot.setPosition(38);
//                griperSev.setPosition(180);
//                depositstate = DeliveryState.transfer;
//            },
//            () -> true
//    );

//    public LambdaCommand clippingPosition = new LambdaCommand(
//            () -> System.out.println("init"),
//            () -> {
//                mainPivot.setPosition(190);
//                secondPivot.setPosition(170);
////                griperSev.setPosition(180);
////                depositstate = deposit.basket;
//            },
//            () -> true
//    );


//    public LambdaCommand cliping = new LambdaCommand(
//            () -> System.out.println("init"),
//            () -> {
//                mainPivot.setPosition(300);
//                secondPivot.setPosition(60);
//                griperSev.setPosition(180);
//                depositstate = DeliveryState.transfer;
//            },
//            () -> true
//    );

//    public LambdaCommand fronCliping = new LambdaCommand(
//            () -> System.out.println("init"),
//            () -> {
//                mainPivot.setPosition(189);
//                secondPivot.setPosition(200);
//                griperSev.setPosition(180);
//                depositstate = DeliveryState.transfer;
//            },
//            () -> true
//    );

    private Command PreClip = new Execute(
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

    public Command Clip = new LambdaCommand(
            () -> {},
            () -> {

                if (fourbarState == fourBarState.grabNest && slideMotor.getCurrentPosition() > 300) {
                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotPreClip)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondPreClip)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.preClip;
                    gripperState = gripper.grab;

                    PreClip.execute();
                } else if (fourbarState == fourBarState.preClip && slideMotor.getCurrentPosition() > 300) {
                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotClip)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondClip)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.clip;
                    gripperState = gripper.grab;

                    Clipping.execute();
                }else if(fourbarState == fourBarState.clip && slideMotor.getCurrentPosition() > 300 && gripperState == gripper.grab){
                    fourBarTimer.reset();
                    ClippingWaitTime = 200;
                    gripperState = gripper.drop;

                }else if(fourbarState == fourBarState.clip && slideMotor.getCurrentPosition() > 300 && gripperState == gripper.drop){
                    fourBarTimer.reset();
                    ClippingWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotBehindTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondBehindTransfer)*microRoboticTime);
                    fourbarState = fourBarState.transferringStates;
                    fourBarTargetState = fourBarState.behindNest;

                    behindNest.execute();
                }

                if (fourbarState == fourBarState.transferringStates && fourBarTimer.milliseconds() > ClippingWaitTime){
                    fourbarState = fourBarTargetState;
                }
            },
            () -> fourbarState != fourBarState.transferringStates
    );


    public Command slideSetPonts(double targetPozition){
        profile.generateMotionProfile(targetPozition, slideMotor.getCurrentPosition());
        if (targetPozition == 0){
            retracting = true;
            counter = 0;
        }
        return followMotionPro;
    }

    @Override
    public void init() {
        slideMotor = getOpModeEX().hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2 = getOpModeEX().hardwareMap.get(DcMotor.class, "slideMotor2");
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mainPivot.initServo("mainPivot",getOpModeEX().hardwareMap);
        secondPivot.initServo("secondPivot",getOpModeEX().hardwareMap);
//        linierRail.initServo("linearRail",getOpModeEX().hardwareMap);
        griperSev.initServo("devClaw",getOpModeEX().hardwareMap);

        slidesReset = getOpModeEX().hardwareMap.get(TouchSensor.class, "DeliveryReset");

        clawSensor = getOpModeEX().hardwareMap.get(TouchSensor.class, "clawsensor   ");


        griperSev.setRange(new PwmControl.PwmRange(500, 2500),180);
        mainPivot.setRange(335);
        secondPivot.setRange(335);

        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        griperSev.setPosition(180);

        mainPivot.setPosition(mainPivotBehindTransfer);
        secondPivot.setPosition(secondBehindTransfer);

        Deposit.execute();

        profile.isVertical(true);

    }

    @Override
    public void execute() {



//        Deposit.execute();

        executeEX();
        if(slides==slideState.moving){
            double slidePower = profile.followProfile(slideMotor.getCurrentPosition());
            slideMotor.setPower(slidePower);
            slideMotor2.setPower(slidePower);
        }

        if (gripperState == Delivery.gripper.grab){
            griperSev.setPosition(0);
        } else if (gripperState == Delivery.gripper.drop) {
            griperSev.setPosition(90);
        }

        if (getCurrentCommand() != followMotionPro){
            if (Math.abs(slideMotor.getCurrentPosition())>200){
                slideMotor.setPower(0.00005);
            }else if(Math.abs(slideMotor.getCurrentPosition())>1000){
                slideMotor.setPower(0.00055);
            }else if(Math.abs(slideMotor.getCurrentPosition())>1400){
                slideMotor.setPower(0.00075);
            }else if(Math.abs(slideMotor.getCurrentPosition())>2000){
                slideMotor.setPower(0.0008);
            }else if(slideMotor.getCurrentPosition() > 15 && slideMotor.getCurrentPosition() < 120 && retracting){
                slideMotor.setPower(-1);
                counter++;
                if(counter == 5){
                    retracting = false;
                }
            }else {
                slideMotor.setPower(0);
            }
        }
    }

    public void genProfile (double slideTarget){
        if (slideTarget == 0){
            retracting = true;
            counter = 0;
        }
        profile.generateMotionProfile(slideTarget, slideMotor.getCurrentPosition());
    }

    public void disableServos(){
        mainPivot.disableServo();
        secondPivot.disableServo();
        griperSev.disableServo();
//        linierRail.disableServo();
    }

    public void safePositions(){
        mainPivot.setPosition(85);
        secondPivot.setPosition(258);
        griperSev.setPosition(90);
    }

}









