package dev.weaponboy.command_library.OpmodeEX;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.weaponboy.command_library.Scheduler;
import dev.weaponboy.command_library.Testing_ControlSystem.SchedulerTest;

public abstract class OpModeEX extends OpMode {

    public Scheduler scheduler = new Scheduler(this);

    public abstract void initEX();

    public abstract void loopEX();

    @Override
    public void init() {
        scheduler.init();
        initEX();
    }

    @Override
    public void loop() {
        scheduler.execute();
        loopEX();
    }

    /**
     * Use this to write data to the log
     * */
    @Override
    public void stop() {
        super.stop();
    }
}
