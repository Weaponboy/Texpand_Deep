package dev.weaponboy.command_library.Subsystems.Examples;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class Template extends SubSystem {

    //hardware declaration goes here

    public Template(OpModeEX opModeEX){
        registerSubsystem(opModeEX, defaultCommand);
    }

    @Override
    public void init() {
        // init code goes here
    }

    @Override
    public void execute() {
        //this executes the current command
        executeEX();
        //any other loop/update code goes here
    }

    //command template
    //first lambda is run once when the command is started
    //second lambda is run until the command completes
    //third lambda is the command completion condition
    LambdaCommand defaultCommand = new LambdaCommand(
            () -> System.out.println("init"),
            () -> System.out.println("execute"),
            () -> true
    );

}
