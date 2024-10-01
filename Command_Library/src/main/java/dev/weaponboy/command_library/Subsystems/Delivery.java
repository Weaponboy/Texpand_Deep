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

public class Delivery extends SubSystem {

   public DcMotor slideMotor;

    double test = 0;
    double targetPosition = 50;
    double currentPosition = 0;

    double maxAccel = 1400;
    double maxVelocity = 140;
    double acceldistance = (maxVelocity * maxVelocity) / (maxAccel*2);
    public ServoDegrees griperSev =new ServoDegrees();
    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();
    public ServoDegrees linierRail= new ServoDegrees();

    ArrayList<Double> motionProfile = new ArrayList<>();
    ArrayList<Double> positions = new ArrayList<>();
    ArrayList<Double> time = new ArrayList<>();
    ElapsedTime currentTime = new ElapsedTime();
    ElapsedTime flipBackTimer = new ElapsedTime();

    int lastIndex = 0;
    double slidetime;
    double veloToMotorPower = 1/maxVelocity;

    public final int maxSlideHeight = 1720;
    public final int maxSlideHeightCM = 53;
    private final int ticksPerCm = maxSlideHeight / maxSlideHeightCM;
    public final double CMPerTick = (double) maxSlideHeightCM / maxSlideHeight;

    double topRailFullExtension = 0;
    double topRailAllTheWayIn = 335;

    public final double highBasket = 600 * CMPerTick;
    public final double lowBasket = 200 * CMPerTick;

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
            () -> {
                currentTime.reset();
                lastIndex = 0;
            },
            ()-> {

                if (lastIndex >= time.size()){
                    while (positions.get(lastIndex-time.size()) < slideMotor.getCurrentPosition()*CMPerTick){
                        lastIndex++;
                    }
                }else {
                    if (lastIndex < time.size()){
                        if (time.get(lastIndex) < currentTime.milliseconds()) {
                            lastIndex++;
                        }
                    }
                }

                double targetVelocity = motionProfile.get(lastIndex);
                double targetMotorPower = targetVelocity*veloToMotorPower;

                slideMotor.setPower(targetMotorPower);
                System.out.println("slideMotor" + targetMotorPower);
                System.out.println("slideMotor" + lastIndex );
                System.out.println("slideMotor" + motionProfile.size());
                if (lastIndex >= motionProfile.size()-1){
                    slidesState =slideState.holdPosition;

                }

            },
            ()-> lastIndex >= motionProfile.size()-1
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
        setDefaultCommand(nothing);
//        drop.execute();

    }

    @Override
    public void execute() {
        executeEX();
        if(slidesState==slideState.holdPosition){
            queueCommand(holdPosition);
        }
    }

    public void genProfile (double slideTarget){

        time.clear();
        motionProfile.clear();
        slidetime = 0;
        targetPosition = slideTarget;
        currentPosition = slideMotor.getCurrentPosition()*CMPerTick;

        double distanceToTarget = targetPosition - currentPosition;

        double halfwayDistance = targetPosition / 2;
        double newAccelDistance = acceldistance;

        int decelCounter = 0;

        double baseMotorVelocity = (maxVelocity) * 0.15;

        if (acceldistance > halfwayDistance){
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = Math.sqrt(2 * maxAccel * newAccelDistance);

        System.out.println("acceleration_distance: " + acceldistance);
        System.out.println("newMaxVelocity: " + newMaxVelocity);
        System.out.println("newAccelDistance accel: " + newAccelDistance);

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (newAccelDistance > i && targetPosition > currentPosition) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                AccelSlope = ((100 - AccelSlope) * 0.01);

                targetVelocity = (newMaxVelocity * AccelSlope) + baseMotorVelocity;

                if(targetVelocity != 0){
                    slidetime += (1 / targetVelocity) * 1000;
                }

                time.add(slidetime);

                System.out.println("targetVelocity accel: " + targetVelocity);

            }else if (i + newAccelDistance > Math.abs(targetPosition - currentPosition) && targetPosition < currentPosition) {

                decelCounter++;

                int range = (int) Math.abs(newAccelDistance - decelCounter);

                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                DeccelSlope = DeccelSlope * 0.01;

                targetVelocity = (newMaxVelocity * DeccelSlope) + baseMotorVelocity;

                positions.add((double) i+1);

                System.out.println("targetVelocity dccel: " + targetVelocity);

            } else {

                targetVelocity = newMaxVelocity;

                positions.add((double) i+1);

                System.out.println("targetVelocity: " + targetVelocity);

            }

            motionProfile.add(targetVelocity);

        }

        System.out.println("slide time: " + time.size());
        System.out.println("motion profile: " + motionProfile.size());
        System.out.println("positions size: " + positions.size());
        for (int i = 0; i < positions.size(); i++){
            System.out.println("positions profile: " + positions.get(i));
        }

    }

    public void genProfile (double slideTarget, FileWriter fWriter) throws IOException {

        System.out.println("init Second");
        double test = 0;
        time.clear();
        motionProfile.clear();
        slidetime = 0;
        targetPosition = slideTarget;
        double halfwayDistance = targetPosition / 2;
        double newAccelDistance = acceldistance;
        int decelCounter = 0;

        double baseMotorVelocity = 105;

//            if (slideTarget > slideMotor.getCurrentPosition()*CMPerTick){
//                newAccelDistance = targetPosition;
//            }else {
//            }

        if (newAccelDistance > halfwayDistance){
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = Math.sqrt((2 * maxAccel)*newAccelDistance);

        System.out.println("acceldistance" + halfwayDistance);
        System.out.println("newMaxVelocity" + newMaxVelocity);
        System.out.println("newAccelDistance accel" + newAccelDistance);

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (newAccelDistance > i && targetPosition > slideMotor.getCurrentPosition()*CMPerTick) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                AccelSlope = ((100 - AccelSlope) * 0.01);

                targetVelocity = (newMaxVelocity * AccelSlope);

                if(targetVelocity != 0){
                    slidetime += (1 / targetVelocity) * 1000;
                }

                targetVelocity += baseMotorVelocity;

                fWriter.write( "slide time accel: " + slidetime);
                fWriter.write( "targetVelocity accel: " + targetVelocity);
                fWriter.write(System.lineSeparator());

            }else if (i + newAccelDistance > Math.abs(targetPosition - currentPosition) && targetPosition < slideMotor.getCurrentPosition()*CMPerTick) {

                decelCounter++;

                int range = (int) Math.abs(newAccelDistance - decelCounter);

                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                DeccelSlope = DeccelSlope * 0.01;

                targetVelocity = newMaxVelocity * DeccelSlope;

                if(targetVelocity != 0){
                    slidetime += (1 / targetVelocity) * 1000;
                }

                fWriter.write( "slide time decel: " + slidetime);
                fWriter.write( "targetVelocity decel: " + targetVelocity);
                fWriter.write(System.lineSeparator());

            } else {
                targetVelocity = newMaxVelocity;

                if(targetVelocity != 0){
                    slidetime += (1 / targetVelocity) * 1000;
                }

                fWriter.write( "slide time:" + slidetime);
                fWriter.write( "targetVelocity:" + targetVelocity);
                fWriter.write(System.lineSeparator());

            }

            motionProfile.add(targetVelocity);
            time.add(slidetime);
        }

        fWriter.write( "slide time:" + slidetime);
        fWriter.write( "motionprofile.size()" + motionProfile.size());
        fWriter.write(System.lineSeparator());

        fWriter.flush();
    }

    public void disableServos(){
        mainPivot.disableServo();
        secondPivot.disableServo();
        griperSev.disableServo();
        linierRail.disableServo();
    }

}









