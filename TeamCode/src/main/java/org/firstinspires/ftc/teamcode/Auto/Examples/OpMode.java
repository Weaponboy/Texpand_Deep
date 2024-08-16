package org.firstinspires.ftc.teamcode.Auto.Examples;

import dev.weaponboy.command_library.OpmodeEX.OpModeEX;

public class OpMode extends OpModeEX {

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {

        scheduler.driveBase.queueCommand(scheduler.driveBase.PIDControl);

        scheduler.driveBase.overrideCurrent(true, scheduler.driveBase.PIDControl);

        requestOpModeStop();

    }

}
