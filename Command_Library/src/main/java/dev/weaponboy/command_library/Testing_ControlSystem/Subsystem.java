package dev.weaponboy.command_library.Testing_ControlSystem;

import dev.weaponboy.command_library.Commands.LambdaCommand;
import dev.weaponboy.command_library.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystem.SubSystem;

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
