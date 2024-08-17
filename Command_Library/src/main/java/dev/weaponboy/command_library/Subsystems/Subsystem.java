package dev.weaponboy.command_library.Subsystems;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class Subsystem extends SubSystem {

    public int test = 0;

    public Subsystem(OpModeEX opModeEX){
        registerSubsystem(opModeEX, defaultCommand);
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {
        executeEX();
        test++;
    }

    LambdaCommand defaultCommand = new LambdaCommand(
            () -> System.out.println("init"),
            () -> System.out.println("execute"),
            () -> test > 5
    );

    public LambdaCommand defaultCommandSecond = new LambdaCommand(
            () -> {
                System.out.println("init Second");
                test = 0;
            },
            () -> System.out.println("execute Second"),
            () -> test > 5
    );

    public LambdaCommand defaultCommandThird = new LambdaCommand(
            () -> {
                System.out.println("init Third");
                test = 0;
            },
            () -> System.out.println("execute Third"),
            () -> test > 5
    );

}
