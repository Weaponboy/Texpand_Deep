package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

public class horizontalSlides extends Subsystem {

DcMotorEx horizontalMotor;

double milimetersPerTick =2.42;

    @Override
    public void execute() {
        executeEX();
    }
    @Override
    public void init() {
        horizontalMotor = getOpModeEX().hardwareMap.get(DcMotorEx.class, "horizontalMotor");
    }
    public horizontalSlides(OpModeEX opModeEX) {
        super(opModeEX);
    }
    LambdaCommand defaultCommand = new LambdaCommand(
            () -> System.out.println("init"),
            () -> System.out.println("execute"),
            () -> true
    );
}
