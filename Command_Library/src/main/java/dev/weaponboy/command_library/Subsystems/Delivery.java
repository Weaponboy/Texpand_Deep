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

    motionProfile profile = new motionProfile(1400, 140, 54, 1720, 0.15);

    double topRailFullExtension = 0;
    double topRailAllTheWayIn = 335;

    public final double highBasket = 600;
    public final double lowBasket = 200;

    ElapsedTime transferTimer = new ElapsedTime();
    boolean flipBackTime;
    boolean gripTimer;

    /**
     * behind transfer position values
     * */
    double mainPivotBehindTransfer = 90;
    double secondPivotTransInt = 190;
    double rotateTransInt = 135;

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
        basketDeposit
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

    public enum clipping{
        drop,
        in
    }

    public clipping getClippingState() {
        return clippingState;
    }

    public void setClippingState(clipping clippingState) {
        this.clippingState = clippingState;
    }

    private clipping clippingState = clipping.drop;

    public Delivery.DeliveryState depositstate = DeliveryState.stowed;

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
                    slideMotor.setPower(0.18);
                }else{
                    slideMotor.setPower(0);
                }
            },
            () -> true
    );

    public Command behindNest = new Execute(
            () -> {
                mainPivot.setPosition(90);
                secondPivot.setPosition(226);
                griperSev.setPosition(180);
                depositstate = DeliveryState.transfer;
            }
    );

    public Command behindTransfer = new Execute(
            () -> {
                mainPivot.setPosition(80);
                secondPivot.setPosition(226);
                griperSev.setPosition(180);
                depositstate = DeliveryState.transfer;
            }
    );

   public LambdaCommand transfer = new LambdaCommand(
            () -> {},
            () -> {
                secondPivot.setPosition(261);
                mainPivot.setPosition(99);
                griperSev.setPosition(180);
                depositstate = DeliveryState.transfer;
            },
            () -> true
    );

    public LambdaCommand BehindTransfer = new LambdaCommand(
            () -> {

            },
            () -> {
                secondPivot.setPosition(258);
                griperSev.setPosition(90);
                mainPivot.setPosition(80);
                depositstate = DeliveryState.transfer;

            },
            () -> true
    );

   public LambdaCommand dropOff = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(300);
                secondPivot.setPosition(38);
                griperSev.setPosition(180);
                depositstate = DeliveryState.transfer;
            },
            () -> true
    );

    public LambdaCommand clippingPosition = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(190);
                secondPivot.setPosition(170);
//                griperSev.setPosition(180);
//                depositstate = deposit.basket;
            },
            () -> true
    );


    public LambdaCommand cliping = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(300);
                secondPivot.setPosition(60);
                griperSev.setPosition(180);
                depositstate = DeliveryState.transfer;
            },
            () -> true
    );

    public LambdaCommand fronCliping = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(189);
                secondPivot.setPosition(200);
                griperSev.setPosition(180);
                depositstate = DeliveryState.transfer;
            },
            () -> true
    );


    public LambdaCommand followMotionPro = new LambdaCommand(
            () ->{},
            ()-> slideMotor.setPower(profile.followProfile(slideMotor.getCurrentPosition())),
            ()-> profile.isSlideRunning()
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
        behindTransfer.execute();
        griperSev.setPosition(90);
//        drop.execute();

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

        if (clippingState == Delivery.clipping.in){
            linierRail.setPosition(0);
        } else if (clippingState == Delivery.clipping.drop) {
            linierRail.setPosition(270);
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









