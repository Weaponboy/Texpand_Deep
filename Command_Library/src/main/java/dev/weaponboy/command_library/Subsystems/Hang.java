package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class Hang  extends SubSystem {

    Servo hang1;
    Servo hang2;

    public Hang(OpModeEX opModeEX){
        registerSubsystem(opModeEX, stop);
    }

    @Override
    public void init() {
        hang1 = getOpModeEX().hardwareMap.get(Servo.class,"hang1");
        hang2 = getOpModeEX().hardwareMap.get(Servo.class,"hang2");
        hang1.setDirection(Servo.Direction.REVERSE);

        hang1.setPosition(0.5);
        hang2.setPosition(0.5);
    }

    @Override
    public void execute() {
        executeEX();
    }

    public LambdaCommand stop = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                hang1.setPosition(0.5);
                hang2.setPosition(0.5);
            },
            () -> true
    );

    public LambdaCommand in = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                hang1.setPosition(1);
                hang2.setPosition(1);
            },
            () -> true
    );

    public LambdaCommand out = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                hang1.setPosition(0);
                hang2.setPosition(0);
            },
            () -> true
    );
}
