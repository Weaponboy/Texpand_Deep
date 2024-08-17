package dev.weaponboy.command_library.CommandLibrary.OpmodeEX;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Subsystems.DriveBase;

public class Scheduler {

    OpModeEX opModeEX;

    SubSystem[] subSystems;

    public Scheduler(OpModeEX opModeEX, SubSystem[] subSystems){
        this.opModeEX = opModeEX;
        this.subSystems = subSystems;
    }

    protected void init(){
        for (int i = 0; i <= subSystems.length-1; i++){
            subSystems[i].init();
        }
    }

    protected void execute(){
        for (int i = 0; i <= subSystems.length-1; i++){
            subSystems[i].execute();
        }
    }

}
