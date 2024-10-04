package dev.weaponboy.command_library.Subsystems;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

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


    public enum deposit {
        preTransFer,
        transfer,
        postTransfer,
        basket,
    }

    public Delivery.deposit depositstate = deposit.preTransFer;

    public enum slideState {
        holdPosition,
        moving,
    }

    public Delivery.slideState slidesState = slideState.holdPosition;


    public Delivery(OpModeEX opModeEX) {
        registerSubsystem(opModeEX,holdPosition);
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

    public LambdaCommand nothing = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
//                if (slideMotor.getCurrentPosition() < 130){
//                    slideMotor.setPower(0);
//                }else {
//                    slideMotor.setPower(0.36);
//                }
            },

            () -> true

    );

   public LambdaCommand drop = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                griperSev.setPosition(110);
            },
            () -> true
    );

   public LambdaCommand grip = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                griperSev.setPosition(180);
            },
            () -> true
    );

   public LambdaCommand transfer = new LambdaCommand(
            () -> {

            },
            () -> {
                secondPivot.setPosition(261);
                mainPivot.setPosition(99);
                griperSev.setPosition(180);
                depositstate =deposit.transfer;
            },
            () -> true
    );

    public LambdaCommand behindTransfer = new LambdaCommand(
            () -> {

            },
            () -> {
                secondPivot.setPosition(258);
                griperSev.setPosition(90);
                mainPivot.setPosition(80);
                depositstate =deposit.preTransFer;

            },
            () -> true
    );

   public LambdaCommand dropOff = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(300);
                secondPivot.setPosition(38);
                griperSev.setPosition(180);
                depositstate =deposit.basket;
            },
            () -> true
    );
    public LambdaCommand cliping = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(300);
                secondPivot.setPosition(60);
                griperSev.setPosition(180);
                depositstate =deposit.basket;
            },
            () -> true
    );
    public LambdaCommand fronCliping = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(189);
                secondPivot.setPosition(200);
                griperSev.setPosition(180);
                depositstate =deposit.basket;
            },
            () -> true
    );
    public LambdaCommand postTransfer = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(90);
                secondPivot.setPosition(226);
                griperSev.setPosition(180);
                depositstate =deposit.postTransfer;
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
        secondPivot.setPosition(258);
        mainPivot.setPosition(80);
        griperSev.setPosition(90);
//        drop.execute();

    }

    @Override
    public void execute() {
        executeEX();
//        if(slidesState==slideState.holdPosition){
//            queueCommand(holdPosition);
//        }
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









