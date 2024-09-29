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
    int currentPosition = 0;
    double maxAccel = 3000;
    double maxVelocity = 293;
    double acceldistance = (maxVelocity * maxVelocity) / maxAccel*2;
    public ServoDegrees griperSev =new ServoDegrees();
    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();
    public ServoDegrees linierRail= new ServoDegrees();

    ArrayList<Double> motionprofile = new ArrayList<>();
    ArrayList<Double> time = new ArrayList<>();
    ElapsedTime currentTime = new ElapsedTime();

    int lastIndex = 0;
    double slidetime;
    double veloToMotorPower = 1/maxVelocity;

    public final int maxSlideHeight = 1720;
    public final int maxSlideHeightCM = 53;
    private final int ticksPerCm = maxSlideHeight / maxSlideHeightCM;
    private final double CMPerTick = (double) maxSlideHeightCM / maxSlideHeight;

    double topRailFullExtension = 0;
    double topRailAllTheWayIn = 335;

    public final double highBasket = 600 * CMPerTick;
    public final double lowBasket = 200 * CMPerTick;

    ElapsedTime transferTimer = new ElapsedTime();
    double transferWaitTime;
    double gripTimer;


    public enum deposit {
        preTransFer,
        transfer,
        postTransfer,
        basket,
    }

    public Delivery.deposit depositstate = deposit.preTransFer;


    public Delivery(OpModeEX opModeEX) {
        registerSubsystem(opModeEX,holdPosition);
    }
    public void slideHold (){
        if (Math.abs(slideMotor.getCurrentPosition())>40){
            slideMotor.setPower(0.1);
        }else if(Math.abs(slideMotor.getCurrentPosition())>1000){
            slideMotor.setPower(0.15);
        }else if(Math.abs(slideMotor.getCurrentPosition())>1400){
            slideMotor.setPower(0.18);
        }else{
            slideMotor.setPower(0);
        }
//        if(Math.abs(slideMotor.getCurrentPosition())>250){
//            slideMotor.setPower(0.55);
//        }else if (Math.abs(slideMotor.getCurrentPosition())>150){
//            slideMotor.setPower(0.45);
//        }
//        else if (Math.abs(slideMotor.getCurrentPosition())>80){
//            slideMotor.setPower(0.4);
//        }
//        else if (Math.abs(slideMotor.getCurrentPosition())>20){
//            slideMotor.setPower(0.35);
//        }else{
//            slideMotor.setPower(0);
//        }
    }

    public LambdaCommand holdPosition = new LambdaCommand(
            () -> {},
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
                griperSev.setPosition(190);
            },
            () -> true
    );
   public LambdaCommand transfer = new LambdaCommand(
            () -> {
                transferTimer.reset();
                mainPivot.setPosition(81);
                secondPivot.setPosition(258);
                gripTimer = 200;
                transferWaitTime = 150;
                depositstate =deposit.transfer;
            },
            () -> {
                if (transferTimer.milliseconds() > transferWaitTime){
                    grip.execute();
                    gripTimer = 0;
                }
            },
            () -> gripTimer == 0
    );

    public LambdaCommand behindTransfer = new LambdaCommand(
            () -> {
                drop.execute();
                transferTimer.reset();
                gripTimer = 200;
                transferWaitTime = 80;
                depositstate =deposit.preTransFer;
            },
            () -> {
                if (transferTimer.milliseconds() > transferWaitTime){
                    mainPivot.setPosition(74);
                    secondPivot.setPosition(258);
                    gripTimer = 0;
                }
            },
            () -> gripTimer == 0
    );

   public LambdaCommand dropOff = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(300);
                secondPivot.setPosition(38);
                grip.execute();
                depositstate =deposit.basket;
            },
            () -> true
    );
    public LambdaCommand postTransfer = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                mainPivot.setPosition(85);
                secondPivot.setPosition(229);
                grip.execute();
                depositstate =deposit.postTransfer;
            },
            () -> true
    );


    public LambdaCommand followMotionPro = new LambdaCommand(
            () -> {
                currentTime.reset();
                lastIndex = 0;
                System.out.println("currentTime" + slidetime);
            },
            ()-> {

                while (time.get(lastIndex) < currentTime.milliseconds()){
                    System.out.println("lastIndex" + time.get(lastIndex));
                    System.out.println("currentTime" + currentTime.milliseconds());
                    if (!(lastIndex == time.size()-1)){
                        lastIndex++;
                    }
                }

                double targetVelocity = motionprofile.get(lastIndex);
                double targetMotorPower = targetVelocity*veloToMotorPower;
                slideMotor.setPower(targetMotorPower);
//                System.out.println("currentTime" + currentTime.milliseconds());
                System.out.println("slideMotor" + targetMotorPower);

            },
            ()-> currentTime.milliseconds() > slidetime || (slideMotor.getCurrentPosition() * CMPerTick) > targetPosition
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

//        drop.execute();

    }

    @Override
    public void execute() {
        executeEX();
    }

    public void genProfile (double slideTarget){

            System.out.println("init Second");
            double test = 0;
            time.clear();
            motionprofile.clear();
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

                    System.out.println("slidetime accel " + slidetime);

                    System.out.println("targetVelocity accel " + targetVelocity);

                }else if (i + newAccelDistance > Math.abs(targetPosition - currentPosition) && targetPosition < slideMotor.getCurrentPosition()*CMPerTick) {

                    decelCounter++;

                    int range = (int) Math.abs(newAccelDistance - decelCounter);

                    double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                    DeccelSlope = DeccelSlope * 0.01;

                    targetVelocity = newMaxVelocity * DeccelSlope;

                    if(targetVelocity != 0){
                        slidetime += (1 / targetVelocity) * 1000;
                    }

                    System.out.println("slidetime dccel " + slidetime);

                    System.out.println("targetVelocity dccel " + targetVelocity);

                } else {
                    targetVelocity = newMaxVelocity;

                    if(targetVelocity != 0){
                        slidetime += (1 / targetVelocity) * 1000;
                    }

                    System.out.println("slidetime " + slidetime);

                    System.out.println("targetVelocity" + targetVelocity);

                }

                motionprofile.add(targetVelocity);
                time.add(slidetime);
            }

            System.out.println("slide time" + slidetime);
            System.out.println("slide time" + motionprofile.size());
        }

    public void genProfile (double slideTarget, FileWriter fWriter) throws IOException {

        System.out.println("init Second");
        double test = 0;
        time.clear();
        motionprofile.clear();
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

            motionprofile.add(targetVelocity);
            time.add(slidetime);
        }

        fWriter.write( "slide time:" + slidetime);
        fWriter.write( "motionprofile.size()" + motionprofile.size());
        fWriter.write(System.lineSeparator());

        fWriter.flush();
    }

}









