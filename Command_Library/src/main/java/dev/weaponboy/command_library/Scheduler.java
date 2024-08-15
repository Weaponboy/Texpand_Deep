package dev.weaponboy.command_library;

import dev.weaponboy.command_library.Examples.Subsystems.DriveBase;
import dev.weaponboy.command_library.OpmodeEX.OpModeEX;

public class Scheduler {

    OpModeEX opModeEX;

    public DriveBase driveBase;

    public Scheduler(OpModeEX opModeEX){
        this.opModeEX = opModeEX;
        driveBase = new DriveBase(opModeEX);
    }

    public void init(){
        driveBase.init();
    }

    public void execute(){
        driveBase.execute();
    }

}
