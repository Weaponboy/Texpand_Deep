package dev.weaponboy.command_library.Subsystems;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.Execute;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.command_library.Hardware.motionProfile;

public class Delivery extends SubSystem {

   public DcMotor slideMotor;

    public ServoDegrees griperSev =new ServoDegrees();
    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();
    public ServoDegrees linierRail= new ServoDegrees();

    motionProfile profile = new motionProfile(1400, 140, 95, 2250, 0.15);

    double topRailFullExtension = 0;
    double topRailAllTheWayIn = 335;

    public final double highBasket = 95;
    public final double lowBasket = 200;

    ElapsedTime transferTimer = new ElapsedTime();
    boolean flipBackTime;
    boolean gripTimer;

    /**
     * servo time per degrees
     * */
    double axonMaxTime = (double) 690 / 360;
    double microRoboticTime = (double) 840 / 360;
    double gripperOpenTime = 400;

    /**
     * behind transfer position values
     * */
    double mainPivotBehindTransfer = 26.8;
    double secondBehindTransfer = 226;
    double gripperBehindTransfer = 110;

    /**
     * transfer position values
     * */
    double mainPivotTransfer = 49.8;
    double secondTransfer = 244;
    double gripperTransfer = 180;

    /**
     * bucket deposit position values
     * */

    double mainPivotDepo = 270;
    double secondDepo = 20;
    double gripperDepo = 180;

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
        transferringStates
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


    ElapsedTime fourBarTimer = new ElapsedTime();
    double transferWaitTime;

    public enum slideState {
        holdPosition,
        moving,
    }

    public Delivery.slideState slidesState = slideState.holdPosition;

    public Delivery(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, holdPosition);
    }

    public LambdaCommand holdPosition = new LambdaCommand(
            () -> {},
            () -> {
                if (Math.abs(slideMotor.getCurrentPosition())>90){
                    slideMotor.setPower(0.1);
                }else if(Math.abs(slideMotor.getCurrentPosition())>1000){
                    slideMotor.setPower(0.15);
                }else if(Math.abs(slideMotor.getCurrentPosition())>1400){
                    slideMotor.setPower(0.2);
                }else if(Math.abs(slideMotor.getCurrentPosition())>2000){
                    slideMotor.setPower(0.22);
                }else if(slideMotor.getCurrentPosition() > 5 && slideMotor.getCurrentPosition() < 40){
                    slideMotor.setPower(-0.5);
                }else {
                    slideMotor.setPower(0);
                }
            },
            () -> true
    );

    private Command behindNest = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotBehindTransfer);
                secondPivot.setPosition(secondBehindTransfer);
                griperSev.setPosition(gripperBehindTransfer);
            }
    );

    private Command Transfer = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotTransfer);
                secondPivot.setPosition(secondTransfer);
                griperSev.setPosition(gripperTransfer);
            }
    );

    private Command Deposit = new Execute(
            () -> {
                mainPivot.setPosition(mainPivotDepo);
                secondPivot.setPosition(secondDepo);
                griperSev.setPosition(gripperDepo);
            }
    );

   public LambdaCommand transfer = new LambdaCommand(
            () -> {},
            () -> {

                if (fourbarState == fourBarState.behindNest){

                    fourBarTimer.reset();
//                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
                    transferWaitTime = 500;
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
           () -> {},
           () -> {

               if (fourbarState == fourBarState.basketDeposit && gripperState == gripper.drop){
                   fourBarTimer.reset();
                    transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
//                   transferWaitTime = 500;
                   fourbarState = fourBarState.transferringStates;
                   fourBarTargetState = fourBarState.behindNest;

                   behindNest.execute();
               } else if (fourbarState == fourBarState.grabNest && slideMotor.getCurrentPosition() > 400) {
                   fourBarTimer.reset();
                   transferWaitTime = Math.max(Math.abs(mainPivot.getPositionDegrees()-mainPivotTransfer)*axonMaxTime, Math.abs(secondPivot.getPositionDegrees()-secondTransfer)*microRoboticTime);
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

                if (fourbarState == fourBarState.grabNest && slideMotor.getCurrentPosition() > 300) {
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


    public LambdaCommand followMotionPro = new LambdaCommand(
            () ->{},
            ()-> slideMotor.setPower(profile.followProfile(slideMotor.getCurrentPosition())),
            ()-> !profile.isSlideRunning()
    );

    @Override
    public void init() {
        slideMotor = getOpModeEX().hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mainPivot.initServo("mainPivot",getOpModeEX().hardwareMap);
        secondPivot.initServo("secondPivot",getOpModeEX().hardwareMap);
        linierRail.initServo("linearRail",getOpModeEX().hardwareMap);
        griperSev.initServo("devClaw",getOpModeEX().hardwareMap);


        secondPivot.setRange(new PwmControl.PwmRange(500, 2500), 270);
        griperSev.setRange(new PwmControl.PwmRange(500, 2500),180);
        mainPivot.setRange(335);
        linierRail.setRange(335);
        behindNest.execute();

    }

    @Override
    public void execute() {
        executeEX();
//        if(slidesState==slideState.holdPosition){
//            queueCommand(holdPosition);
//        }

        if (gripperState == Delivery.gripper.grab){
            griperSev.setPosition(110);
        } else if (gripperState == Delivery.gripper.drop) {
            griperSev.setPosition(180);
        }
    }

    public void genProfile (double slideTarget){
        profile.generateMotionProfile(slideTarget, slideMotor.getCurrentPosition());
    }

    public void disableServos(){
        mainPivot.disableServo();
        secondPivot.disableServo();
        griperSev.disableServo();
        linierRail.disableServo();
    }

    public void safePositions(){
        mainPivot.setPosition(85);
        secondPivot.setPosition(258);
        griperSev.setPosition(90);
    }

}









