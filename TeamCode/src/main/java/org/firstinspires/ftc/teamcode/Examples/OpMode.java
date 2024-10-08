package org.firstinspires.ftc.teamcode.Examples;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.DriveBase;

public class OpMode extends OpModeEX {

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {

        driveBase.queueCommand(driveBase.drivePowers(1, 1, 0));

        driveBase.overrideCurrent(true, driveBase.drivePowers(0,0,0));

        requestOpModeStop();

    }

}
