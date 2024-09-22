package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.MotorEx;

public class Odometry extends SubSystem {

    MotorEx leftPod;
    MotorEx rightPod;
    MotorEx backPod;

    double X, Y, Heading;
    int startHeading;

    double lastRightPod, lastLeftPod, lastBackPod;
    double currentRightPod, currentLeftPod, currentBackPod;

    double podTicks = 2000;
    double wheelRadius = 2.4;
    double trackWidth = 16;
    double backPodOffset = 8;

    double ticksPerCM = podTicks / ((2.0 * Math.PI) * wheelRadius);
    double cmPerDegree = ((2.0 * Math.PI) * backPodOffset) / 360;

    public Odometry(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, update);
    }

    public void startPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        this.startHeading = Heading;
    }

    @Override
    public void init() {
        leftPod.initMotor("LF", getOpModeEX().hardwareMap);
        rightPod.initMotor("RF", getOpModeEX().hardwareMap);
        backPod.initMotor("LB", getOpModeEX().hardwareMap);
    }

    @Override
    public void execute() {

    }

    public double X (){
        return X;
    }

    public double Y (){
        return Y;
    }

    public double Heading (){
        return Heading;
    }

    public LambdaCommand update = new LambdaCommand(
            () -> System.out.println("init odometry update"),
            () -> {
                //need to code this
                //prob some constant accel loc code
            },
            () -> false
    );

    public LambdaCommand updateLineBased = new LambdaCommand(
            () -> System.out.println("init odometry update line based"),
            () -> {
                lastBackPod = currentBackPod;
                lastLeftPod = currentLeftPod;
                lastRightPod = currentRightPod;

                currentBackPod = backPod.getCurrentPosition();
                currentLeftPod = leftPod.getCurrentPosition();
                currentRightPod = rightPod.getCurrentPosition();

                double deltaRight = currentRightPod - lastRightPod;
                double deltaLeft = currentLeftPod - lastLeftPod;
                double deltaBack = currentBackPod - lastBackPod;

                double deltaHeading = ticksPerCM * (deltaLeft - deltaRight) / trackWidth;
                Heading += deltaHeading;

                double deltaX = ticksPerCM * ((deltaLeft + deltaRight)/2);
                double deltaY = (ticksPerCM * deltaBack) - deltaHeading * cmPerDegree;

                X += deltaX * Math.cos(Math.toRadians(Heading)) - deltaY * Math.sin(Math.toRadians(Heading));
                Y += deltaX * Math.sin(Math.toRadians(Heading)) + deltaY * Math.cos(Math.toRadians(Heading));

                updatePodReadings();

            },
            () -> false
    );

    public Command resetPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        startHeading = Heading;
        return resetPosition;
    }

    private final LambdaCommand resetPosition = new LambdaCommand(
            () -> System.out.println("init odometry update"),
            () -> {

            },
            () -> false
    );

    private void updatePodReadings(){
        leftPod.updatePosition();
        rightPod.updatePosition();
        backPod.updatePosition();
    }


}
