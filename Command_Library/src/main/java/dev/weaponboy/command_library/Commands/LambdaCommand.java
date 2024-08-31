package dev.weaponboy.command_library.Commands;

import java.util.function.BooleanSupplier;

public class  LambdaCommand implements Command{

    BooleanSupplier isFinished;
    Runnable init;
    Runnable execute;

    public LambdaCommand(Runnable init, Runnable execute, BooleanSupplier finished){
        this.execute = execute;
        this.init = init;
        this.isFinished = finished;
    }


    @Override
    public void execute() {
        execute.run();
    }

    @Override
    public void init() {
        init.run();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }

}
