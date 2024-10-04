package dev.weaponboy.command_library.CommandLibrary.Commands;

import java.util.function.BooleanSupplier;

public class Execute implements Command{

    Runnable execute;

    public Execute(Runnable execute){
        this.execute = execute;
    }

    @Override
    public void execute() {
        execute.run();
    }

    @Override
    public void init() {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
