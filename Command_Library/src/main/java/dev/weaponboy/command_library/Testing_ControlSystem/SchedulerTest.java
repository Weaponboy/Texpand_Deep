package dev.weaponboy.command_library.Testing_ControlSystem;

import dev.weaponboy.command_library.OpmodeEX.OpModeEX;

public class SchedulerTest {

    OpModeEX opModeEX;

    public Subsystem subsystem;

    public SchedulerTest(OpModeEX opModeEX){
        this.opModeEX = opModeEX;
        subsystem = new Subsystem(opModeEX);
    }

    public void init(){
        subsystem.init();
    }

    public void execute(){
        subsystem.execute();
    }

}
