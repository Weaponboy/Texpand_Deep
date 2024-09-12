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

    public LambdaCommand update = new LambdaCommand(
            () -> System.out.println("init odometry update"),
            () -> {
                //need to code this
                //prob some constant accel loc code
            },
            () -> false
    );

    public Command resetPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        startHeading = Heading;
        return resetPosition;
    }

    private LambdaCommand resetPosition = new LambdaCommand(
            () -> System.out.println("init odometry update"),
            () -> {

            },
            () -> false
    );


}
