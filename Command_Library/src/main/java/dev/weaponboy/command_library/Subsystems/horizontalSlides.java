package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class horizontalSlides extends SubSystem{

    double cmPerTick = 0.242;
    double targetPosition = 0*cmPerTick;
    double currentPosition = 0;
    double maxAccel = 380;
    double maxVelocity = 430;
    double accelDistance = maxVelocity/maxAccel;
    double powerFeedForwardConstant =(1/maxVelocity);
    int lastIndex=0;

    ElapsedTime curentTime= new ElapsedTime();
    ArrayList<Double> motionProfile = new ArrayList<>();
    ArrayList<Double> Time = new ArrayList<>();
    double slideTime;

    DcMotorEx horizontalMotor;
    Servo fourBarMainPivot;
    Servo fourBarSecondPivot;
    Servo griperRotate;
    Servo gripServo;
    Servo linerRailServo;

    public horizontalSlides (){

    }

    @Override
    public void execute() {
        executeEX();
    }

    @Override
    public void init() {
        horizontalMotor = getOpModeEX().hardwareMap.get(DcMotorEx.class, "horizontalMotor");
        horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public horizontalSlides(OpModeEX opModeEX) {
        registerSubsystem(opModeEX,stow);
    }

    Command grip = new LambdaCommand(
            () -> {
            },
            () -> {
                gripServo.setPosition(1);
            },
            () -> true
    );

    Command drop = new LambdaCommand(
            () -> {

            },
            () -> {
                gripServo.setPosition(0.6);
            },
            () -> true
    );


    Command collect = new LambdaCommand(
            () -> {

            },
            () -> {

            },
            () -> true
    );

    Command stow = new LambdaCommand(
            () -> {

            },
            () -> {

            },
            () -> true
    );

    Command transfer = new LambdaCommand(
            () -> {

            },
            () -> {

            },
            () -> true
    );

    LambdaCommand followMotionProfile = new LambdaCommand(
            () -> {
                curentTime.reset();
                lastIndex = 0;
            },
            () -> {
                while (Time.get(lastIndex) < curentTime.milliseconds()) {
                    lastIndex++;
                }

                double targetVelocity=motionProfile.get(lastIndex);
                double targetMotorPower=targetVelocity/powerFeedForwardConstant;
                horizontalMotor.setPower(targetMotorPower);
            },
            () ->slideTime>curentTime.milliseconds()
    );

    public void generateMotionProfile(double slideTarget) {
        this.targetPosition=slideTarget;
        slideTime = 0;
        double halfwayDistance = targetPosition / 2;
        double newAccelDistance = accelDistance;
        int decelCounter = 0;

        if (accelDistance > halfwayDistance) {
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = accelDistance * maxAccel;

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (newAccelDistance > i) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / (double) Math.abs(newAccelDistance) * 100;

                AccelSlope = 100 - (AccelSlope * 0.01);

                targetVelocity = newMaxVelocity * AccelSlope;

                slideTime += (1 / targetVelocity) * 1000;

            }
            if (i + newAccelDistance > Math.abs(targetPosition - currentPosition)) {

                decelCounter++;

                int range = (int) Math.abs(newAccelDistance - decelCounter);

                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                DeccelSlope = DeccelSlope * 0.01;

                targetVelocity = newMaxVelocity * DeccelSlope;

                slideTime += (1 / targetVelocity) * 1000;

            } else {
                targetVelocity = newMaxVelocity;

                slideTime += (1 / targetVelocity) * 1000;
            }

            motionProfile.add(targetVelocity);
            Time.add(slideTime);

        }

    }
}
